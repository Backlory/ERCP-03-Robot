#include "Net/server.h"
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Parser.h>
#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Timespan.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void Require(bool condition, const std::string &message)
{
    if (!condition)
        throw std::runtime_error(message);
}

Poco::JSON::Object::Ptr Request(unsigned short port,
                              const std::string &method,
                              const std::string &path,
                              const std::string &body = "",
                              const std::string &contentType = "application/json")
{
    Poco::Net::HTTPClientSession session("127.0.0.1", port);
    session.setTimeout(Poco::Timespan(5, 0));
    Poco::Net::HTTPRequest request(method, path, Poco::Net::HTTPMessage::HTTP_1_1);
    request.setKeepAlive(false);
    if (method == "POST") {
        request.setContentType(contentType);
        request.setContentLength(static_cast<int>(body.size()));
    }
    session.sendRequest(request) << body;
    Poco::Net::HTTPResponse response;
    auto &stream = session.receiveResponse(response);
    Require(response.getStatus() == Poco::Net::HTTPResponse::HTTP_OK, path + ": HTTP status");
    Poco::JSON::Parser parser;
    return parser.parse(stream).extract<Poco::JSON::Object::Ptr>();
}

void VerifyRoutes(unsigned short port)
{
    const std::vector<std::string> unavailablePaths{
        "/robot/start", "/robot/interrupt", "/robot/skip", "/sensio/action",
        "/arm/status", "/arm/moverel", "/arm/init", "/arm/stop",
        "/beckhoff/read", "/beckhoff/write", "/beckhoff/follow", "/motor"};
    for (const auto &prefix : {std::string(), std::string("/local")}) {
        for (const auto &path : unavailablePaths) {
            for (const auto &method : {"GET", "POST"}) {
                const auto response = Request(port, method, prefix + path,
                                              std::string(method) == "POST" ? "{}" : "");
                Require(!response->getValue<bool>("status"), path + ": must reject request");
                Require(response->getValue<std::string>("info").find("request path") !=
                            std::string::npos,
                        path + ": must fail during route lookup");
                Require(!response->has("data"), path + ": unexpected business response");
            }
        }

        const auto info = Request(port, "GET", prefix + "/robot/info");
        Require(info->getValue<bool>("status"), "robot/info failed");
        const auto modules = info->getObject("data")->getArray("modules");
        Require(modules->size() == 1 && modules->getElement<std::string>(0) == "arm",
                "robot/info module contract changed");

        const auto directory = Request(port, "GET", prefix + "/sensio");
        Require(directory->getValue<bool>("status"), "sensio directory failed");
        const auto channels = directory->getObject("data");
        Require(channels->getArray("sensors")->size() == 5, "sensor directory changed");
        Require(channels->getArray("inputs")->size() == 0, "GPIO input directory changed");
        Require(channels->getArray("outputs")->size() == 3, "GPIO output directory changed");

        const auto action = Request(port, "POST", prefix + "/robot/action",
                                    R"({"type":"arm","action":"invalid-action"})");
        Require(!action->getValue<bool>("status") &&
                    action->getValue<std::string>("info").find("invalid `action`") !=
                        std::string::npos,
                "module action validation changed");

        // 使用不支持的内容类型检查有效路由，校验在设备调用之前终止。
        for (const auto &path : {"/robot/init", "/robot/close", "/robot/status",
                                 "/robot/log", "/robot/forcerecord", "/robot/emergency-stop",
                                 "/settings", "/settings/data", "/settings/update",
                                 "/beckhoff/isopen"}) {
            const auto response = Request(port, "POST", prefix + path, "{}", "text/plain");
            Require(!response->getValue<bool>("status") &&
                        response->getValue<std::string>("info") ==
                            "Only json type `data` is accepted.",
                    std::string(path) + ": route or request validation changed");
        }
    }
}

} // namespace

int main()
{
    // 直接运行生产 HTTP 处理器，通过本机临时端口验证请求与响应。
    Poco::Net::ServerSocket socket(Poco::Net::SocketAddress("127.0.0.1", 0));
    Poco::Net::HTTPServer server(new RequestHandlerFactory(0), socket,
                                 new Poco::Net::HTTPServerParams());
    server.start();
    VerifyRoutes(socket.address().port());
    server.stop();
    std::cout << "PASS: 74 real HTTP requests validated routes, metadata, and request checks.\n";
    return 0;
}
