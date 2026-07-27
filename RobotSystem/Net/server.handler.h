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

/// M1: handler 业务失败时用该键携带失败原因(字符串);
/// handleRequest 将其上提为信封 status=false + info,并从 data 中移除该键。
/// data 内的 status 字段保留,兼容只读内层结果的旧客户端。
inline constexpr char kBusinessFailInfoKey[] = "__fail_info";

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
