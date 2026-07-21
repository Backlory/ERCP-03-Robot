#pragma once
#include <Poco/Net/HTTPServer.h>
#include <Poco/Net/HTTPRequestHandler.h>
#include <Poco/Util/ServerApplication.h>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

class RequestHandlerFactory : public Poco::Net::HTTPRequestHandlerFactory {
public:
    RequestHandlerFactory(int verbose = 1);

    virtual Poco::Net::HTTPRequestHandler *createRequestHandler(
        const Poco::Net::HTTPServerRequest &);

private:
    const int _verbose;
};

class HttpServer : public Poco::Util::ServerApplication {
public:
    HttpServer(int port, int verbose = 1);

protected:
    void initialize(Application &self);
    void uninitialize();
    int main(const std::vector<std::string> &args);

private:
    const int _port;
    const int _verbose;
};
