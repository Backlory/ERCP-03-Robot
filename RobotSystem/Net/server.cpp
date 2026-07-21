#include <Poco/Net/HTTPServerResponse.h>
#include <Poco/Net/HTTPServerRequest.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <boost/make_shared.hpp>
#include <fmt/format.h>
#include "utils.h"
#include "server.h"
#include "server.handler.h"
#include "robot_config.h"

class PageHandler : public Poco::Net::HTTPRequestHandler {
public:
    void handleRequest(Request &req, Response &res)
    {
        Handler::GetInstance().handleRequest(req, res);
    }
};

//-----------------------------------------------------------------------------

RequestHandlerFactory::RequestHandlerFactory(int verbose)
    : _verbose(verbose)
{
}

Poco::Net::HTTPRequestHandler *RequestHandlerFactory::createRequestHandler(const Request &req)
{
    ROBOT_INFO(_verbose > 1,
        "[" << req.getMethod() << "] (" << req.getURI() << ") from " << req.clientAddress())
    return new PageHandler();
}

//-----------------------------------------------------------------------------

HttpServer::HttpServer(int port, int verbose)
    : _port(port)
    , _verbose(verbose)
{
}

void HttpServer::initialize(Application &self)
{
    ServerApplication::loadConfiguration();
    ServerApplication::initialize(self);
}

void HttpServer::uninitialize() { ServerApplication::uninitialize(); }

int HttpServer::main(const std::vector<std::string> &args)
{
    unsigned short port = (unsigned short)config().getInt("HttpServer.port", _port);

    // 1. Bind a ServerSocket with an address
    Poco::Net::ServerSocket serverSocket(port);

    // 2. Pass the ServerSocket to a HttpServer
    Poco::Net::HTTPServer server(
        new RequestHandlerFactory(_verbose), serverSocket, new Poco::Net::HTTPServerParams());

    // 3. Start the HttpServer
    server.start();
    ROBOT_INFO(true, "Server started, listening port " << port)

    // 4. Wait for termination
    waitForTerminationRequest();

    // 5. Stop the HttpServer
    server.stop();
    ROBOT_INFO(true, "Server stopped.")

    return Application::EXIT_OK;
}

//-----------------------------------------------------------------------------

Json Handler::innerHandler(Request &req)
{
    Json ret;

    try {
        // Check req uri
        auto uri = ilsr::split(req.getURI(), "/");
        // Accept "/local" as start base URI
        if (uri.size() > 0 && uri[0] == "local") {
            uri.erase(uri.begin());
        }

        if (uri.size() < 1) {
            ret.set("error", "Empty request path.");
            return ret;
        }
        if (handlers.find(uri[0]) == handlers.end()) {
            ret.set("error", "Not find main request path.");
            return ret;
        }

        if (uri.size() == 1) {
            // Check if 1st uri has `/` node
            if (handlers.at(uri[0]).find("/") == handlers.at(uri[0]).end()) {
                ret.set("error", "Not find the default second request path.");
                return ret;
            }
            uri.emplace_back("/");
        } else if (uri.size() == 2) {
            if (handlers.at(uri[0]).find(uri[1]) == handlers.at(uri[0]).end()) {
                ret.set("error", "Not find second request path.");
                return ret;
            }
        }

        auto &h = handlers.at(uri[0]).at(uri[1]);
        if (!h) {
            ret.set("error", "Internal handler error, ask the developer for help.");
            return ret;
        }

        // Check req if valid
        Poco::JSON::Object::Ptr pobj = nullptr;

        if (req.hasContentLength()) {
            if (req.getContentType() != "application/json") {
                ret.set("error", "Only json type `data` is accepted.");
                return ret;
            }

            std::string src(
                std::istreambuf_iterator<char>(req.stream()), std::istreambuf_iterator<char>());
            Poco::JSON::Parser parser;
            auto json = parser.parse(src);
            pobj = json.extract<Poco::JSON::Object::Ptr>();
        }

        Json obj = pobj ? *pobj : Json();
        auto error = h->checker(obj);
        if (!error.empty()) {
            ret.set("error", "Data check failed: " + error);
            return ret;
        }

        ret = h->handler(obj);

    } catch (std::exception &e) {
        ret.set("error", std::string("Uncatched error: ") + e.what());
    }
    return ret;
}

void Handler::handleRequest(Request &req, Response &res)
{
    // Make feedback by data、
    res.add("Access-Control-Allow-Origin", "*");
    res.add("Access-Control-Allow-Methods", "POST,GET,OPTIONS,DELETE");
    res.add("Access-Control-Max-Age", "3600");
    res.add("Access-Control-Allow-Headers", "x-requested-with,content-type");
    res.add("Access-Control-Allow-Credentials", "true");
    res.setContentType("application/json");

    res.setStatus(Poco::Net::HTTPResponse::HTTP_OK);

    std::ostream &out = res.send();
    Poco::JSON::Object data;
    data.set("req", req.getMethod());
    data.set("uri", req.getURI());

    auto exit = [&out, &data](bool succ, std::string info = "") {
        data.set("status", succ);
        data.set("info", info);
        data.stringify(out);
        out.flush();
    };

    auto ret = innerHandler(req);
    if (ret.has("error")) {
        return exit(false, ret.get("error").extract<std::string>());
    }

    if (ret.size() > 0) {
        if (ret.has("data") && ret.size() == 1) {
            data.set("data", ret.get("data"));
        } else {
            data.set("data", ret);
        }
    }
    return exit(true);
}
