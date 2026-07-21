#pragma once
#include <algorithm>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/signals2.hpp>
namespace fs = boost::filesystem;

#include "dll_export.h"

#if defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <windows.h>
#include <libloaderapi.h>
#define WINDOWS_API
using hdll = HMODULE;
#else
#include <stdio.h>
#include <link.h>
#include <dlfcn.h>
#include <elf.h>
#define LINUX_API
using hdll = void *;
#endif

class DllImporter {

public:
    DllImporter(std::string entry, int verbose = 1)
#if defined(WINDOWS) || defined(_WINDOWS) || defined(WIN32) || defined(_WIN32)
        : entry(fs::current_path() / "mods" / (entry + ".dll"))
#else
        : entry(fs::current_path() / "mods" / ("lib" + entry + ".so"))
#endif
        , verbose(verbose)
    {
        Load();
    }

    DllImporter(const DllImporter &) = delete;

    ~DllImporter()
    {
        if (IsRunning()) {
            Stop();
        }
        Dispose();
        UnLoad();
    }

public:
    bool Load(bool create = false)
    {
        if (!Loaded()) {
#ifdef WINDOWS_API
            dll_handler = LoadLibraryA(entry.string().c_str());
#else
            dll_handler = dlopen(entry.string().c_str(), RTLD_NOW | RTLD_LOCAL);
#endif
            if (!Loaded())
                return false;
            try {
#ifdef WINDOWS_API
                CreateModule = GetMethod<CreateModulePtr>("CreateModule");
                DisposeModule = GetMethod<DisposeModulePtr>("DisposeModule");
                GetModuleVersion = GetMethod<GetModuleVersionPtr>("GetModuleVersion");
#else
                auto entries = get_entries(dll_handler);
                CreateModule = GetMethod<CreateModulePtr>(entries, "CreateModule");
                DisposeModule = GetMethod<DisposeModulePtr>(entries, "DisposeModule");
                GetModuleVersion = GetMethod<GetModuleVersionPtr>(entries, "GetModuleVersion");
#endif
            } catch (std::exception e) {
                UnLoad();
                return false;
            }
        }

        OnLoad();

        if (create) {
            return Create();
        }
        ROBOT_INFO(true, "Loaded dll `" << entry << "`")
        return true;
    }

    void UnLoad()
    {
        auto name = entry.string();
        if (Loaded()) {
#ifdef WINDOWS_API
            if (FreeLibrary(dll_handler)) {
#else
            if (dlclose(dll_handler)) {
#endif
                dll_handler = nullptr;
                ROBOT_INFO(true, "Released dll `" << name << "`")
            }
        }
    }

protected:
#ifdef WINDOWS_API
    template <typename T>
    T GetMethod(const std::string &name, bool nullable = false)
    {
        if (!Loaded()) {
            return nullptr;
        }
        auto addr = GetProcAddress(dll_handler, name.c_str());
        if (!addr && !nullable)
            throw std::runtime_error("Load dll failed.");
        return addr ? reinterpret_cast<T>(addr) : nullptr;
    }
#else

    static std::vector<std::string> get_entries(void *dll)
    {
        std::vector<std::string> entry;

        struct link_map *map = nullptr;
        if (dlinfo(dll, RTLD_DI_LINKMAP, &map) != 0) {
            return entry;
        }

        Elf64_Sym *symtab = nullptr;
        char *strtab = nullptr;
        int symentries = 0;
        for (auto section = map->l_ld; section->d_tag != DT_NULL; ++section) {
            if (section->d_tag == DT_SYMTAB) {
                symtab = (Elf64_Sym *)section->d_un.d_ptr;
            }
            if (section->d_tag == DT_STRTAB) {
                strtab = (char *)section->d_un.d_ptr;
            }
            if (section->d_tag == DT_SYMENT) {
                symentries = section->d_un.d_val;
            }
        }
        int size = strtab - (char *)symtab;
        for (int k = 0; k < size / symentries; ++k) {
            auto sym = &symtab[k];
            // If sym is function
            if (ELF64_ST_TYPE(symtab[k].st_info) == STT_FUNC) {
                // str is name of each symbol
                auto str = &strtab[sym->st_name];
                entry.push_back(str);
            }
        }
        return entry;
    }

    static std::vector<std::string> filter_entry(
        const std::vector<std::string> &entries, const std::string &name, bool greed = true)
    {
        bool found = false;
        std::vector<std::string> filtered;
        std::copy_if(entries.begin(), entries.end(), std::back_inserter(filtered),
            [name, greed, &found](const std::string &e) {
                return !(found && greed) && (e.find(name) != std::string::npos);
            });
        return filtered;
    }

    template <typename T>
    T GetMethod(
        const std::vector<std::string> &entry, const std::string &name, bool nullable = false)
    {
        if (!Loaded()) {
            return nullptr;
        }

        auto names = filter_entry(entry, name);
        if (names.size() <= 0) {
            throw std::runtime_error("Not found function entry.");
        }
        // if (names.size() > 1) {
        //     throw std::runtime_error("Multi function entries found.");
        // }
        auto addr = dlsym(dll_handler, names[0].c_str());
        if (!addr && !nullable)
            throw std::runtime_error("Load dll failed.");
        return addr ? reinterpret_cast<T>(addr) : nullptr;
    }

#endif
public:
    navi::ModPtr GetModule()
    {
        //
        return mod_instance;
    }

    std::string GetEntry() { return entry.stem().string(); }

    bool Create()
    {
        if (Loaded()) {
            CreateModule(mod_instance);
            return mod_instance != nullptr;
        }
        return false;
    }

    bool Dispose()
    {
        if (Loaded()) {
            if (DisposeModule(mod_instance)) {
                mod_instance = nullptr;
            }
        }
        return mod_instance == nullptr;
    }

    bool Start()
    {
        if (Created()) {
            return mod_instance->Start();
        }
        return false;
    }

    bool Stop()
    {
        if (Created()) {
            return mod_instance->Stop();
        }
        return false;
    }

public:
    std::string GetVersion() const { return Loaded() ? GetModuleVersion() : ""; }

    bool Loaded() const { return dll_handler != nullptr; }

    bool Created() const { return mod_instance != nullptr; }

    bool Exists() const { return fs::exists(entry); }

    bool IsRunning() const { return Created() ? mod_instance->IsRunning() : false; }

protected:
    boost::signals2::signal<void(void)> OnLoad;

    typedef bool (*CreateModulePtr)(navi::ModPtr &);
    typedef bool (*DisposeModulePtr)(navi::ModPtr);
    typedef char *(*GetModuleVersionPtr)();

    CreateModulePtr CreateModule;
    DisposeModulePtr DisposeModule;
    GetModuleVersionPtr GetModuleVersion;

protected:
    const fs::path entry;
    const int verbose;
    hdll dll_handler = nullptr;
    navi::ModPtr mod_instance = nullptr;
};
