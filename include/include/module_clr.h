#pragma once
#include <string>

class IModule_cpp {
public:
    virtual std::string Name() = 0;
    virtual std::wstring NameReadable() = 0;
    virtual std::string NameAbbr() = 0;
    virtual std::string Version() = 0;
    virtual std::string Icon() = 0;
    virtual bool Create() = 0;
    virtual bool Destroy() = 0;
    virtual bool Start() = 0;
    virtual bool Stop() = 0;
    virtual bool IsRunning() = 0;
    virtual bool IsCreated() = 0;
};
