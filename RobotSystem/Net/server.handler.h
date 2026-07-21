#pragma once
#include <map>
#include <memory>
#include <set>
#include <typeindex>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Net/HTTPServerResponse.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/Net/HTTPRequestHandler.h>

using Request = Poco::Net::HTTPServerRequest;
using Response = Poco::Net::HTTPServerResponse;
using Json = Poco::JSON::Object;

using type_list = std::set<std::type_index>;

struct keys_info {
    std::string name; /// json member name
    type_list types; /// acceptable json member types
    bool optional; /// is the json member optional
    std::vector<keys_info> sub_infos; /// json member's members
};

class request_case {
public:
    /// <summary>
    /// Check if the request json data valid.
    /// </summary>
    /// <returns>Error info, empty means correct.</returns>
    virtual std::string checker(const Json &) = 0;

    /// <summary>
    /// Handle the request and return need data.
    /// </summary>
    /// <returns>The returned data.</returns>
    virtual Json handler(const Json &) = 0;

protected:
    std::vector<keys_info> info;
};

class Handler {
public:
    static Handler &GetInstance()
    {
        static Handler h;
        return h;
    }

    void handleRequest(Request &req, Response &res);

protected:
    Handler();
    Handler(const Handler &) = delete;

    Json innerHandler(Request &req);

private:
    std::map<std::string, std::map<std::string, std::shared_ptr<request_case>>> handlers;
};
