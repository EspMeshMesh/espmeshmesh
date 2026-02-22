#include <espmeshmesh.h>
#include <log.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

static const char *TAG = "MAIN";
static std::atomic<bool> gRunning{true};

static void sigintHandler(int) {
    gRunning = false;
}

static void logCb(int level, const char *tag, int line, const char *format, va_list args) {
    char msg[256] = {0};
    vsnprintf(msg, sizeof(msg), format, args);
    std::cout << "[" << tag << "](" << line << ") " << msg << std::endl;
}

static void printLog(int level, const char *tag, int line, const char *format, ...) {
    va_list args;
    va_start(args, format);
    logCb(level, tag, line, format, args);
    va_end(args);
}

int main() {
    printLog(3, TAG, __LINE__, "EspMeshMesh Native Application", nullptr);


    espmeshmesh::setLibLogCb(logCb);
    
    // Esempio: crea un'istanza di EspMeshMesh
    espmeshmesh::EspMeshMesh mesh;
    printLog(3, TAG, __LINE__, "Libreria versione: %s", mesh.libVersion().c_str());

    espmeshmesh::EspMeshMesh::SetupConfig config = {
        .hostname = "espmeshmesh-app",
        .wifi = {
            .interface = "wlp0s20f0u3u4u2",
            .channel = 3,
            .txPower = 10,    
        },
        .uart = {
            .baudRate = 0,
            .txBuffer = 0,
            .rxBuffer = 0,
        },
        .nodeType = espmeshmesh::EspMeshMesh::NodeType::ESPMESH_NODE_TYPE_BACKBONE,
        .fwVersion = "1.0.0",
        .compileTime = __DATE__ " " __TIME__,
    };

    mesh.setup(&config);
    mesh.dumpConfig();

    std::signal(SIGINT, sigintHandler);

    while(gRunning) {
        mesh.loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    mesh.shutdown();
    printLog(3, TAG, __LINE__, "Tearing down the mesh");

    auto startTime = std::chrono::system_clock::now();
    while(true) {
        if(mesh.teardown()) {
            break;
        }
        if(std::chrono::system_clock::now() - startTime > std::chrono::seconds(2)) {
            std::cout << "Timeout while tearing down the mesh" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    printLog(3, TAG, __LINE__, "Exiting the application");
    return 0;
}
