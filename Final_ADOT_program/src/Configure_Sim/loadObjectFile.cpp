#include "loadObjectFile.h"

using namespace std;

void* loadObjectFile(string fileName, void* &handle)
{

    handle = dlopen(fileName.c_str(), RTLD_LAZY);
    if (!handle) {
        throw runtime_error("Error opening object file: " + string(dlerror()));
    }

    dlerror(); // clear the error
    
    // Get function to construct model
    void* func = dlsym(handle, "constructModel");
    char* error = dlerror();
    if (error != nullptr || func == nullptr) {
        throw runtime_error("Error locating function \"testModel constructModel()\" in object file: " + string(error));
    }

    return func;
}