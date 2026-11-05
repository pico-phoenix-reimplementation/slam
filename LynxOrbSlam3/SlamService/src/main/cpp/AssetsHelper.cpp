#include "AssetsHelper.h"


AssetsHelper::AssetsHelper(const std::string& internalDataPath, AAssetManager* aAssetManager) :
    internalDataPath(internalDataPath), aAssetManager(aAssetManager) { }

char* AssetsHelper::generateDataFilePath(const char* fileName){
    size_t pathLen = internalDataPath.size() + 1 + strlen(fileName) + 1;
    char* destPath = (char*) malloc(pathLen);
    if (destPath == NULL) {
        return NULL;
    }
    snprintf(destPath, pathLen, "%s/%s", internalDataPath.c_str(), fileName);

    return destPath;
}


char* AssetsHelper::copyAssetToInternalStorage(const char* assetName){
    if (aAssetManager == NULL || assetName == NULL) {
        return NULL;
    }

    AAssetManager* mgr = aAssetManager;
    if (mgr == NULL) {
        return NULL;
    }

    AAsset* asset = AAssetManager_open(mgr, assetName, AASSET_MODE_STREAMING);
    if (asset == NULL) {
        return NULL;
    }

    off_t assetLength = AAsset_getLength(asset);
    if (assetLength <= 0) {
        AAsset_close(asset);
        return NULL;
    }

    char* buffer = (char*) malloc(assetLength);
    if (buffer == NULL) {
        AAsset_close(asset);
        return NULL;
    }

    int bytesRead = AAsset_read(asset, buffer, assetLength);
    AAsset_close(asset);
    if (bytesRead != assetLength) {
        free(buffer);
        return NULL;
    }

    size_t pathLen = internalDataPath.size() + 1 + strlen(assetName) + 1;
    char* destPath = (char*) malloc(pathLen);
    if (destPath == NULL) {
        free(buffer);
        return NULL;
    }
    snprintf(destPath, pathLen, "%s/%s", internalDataPath.c_str(), assetName);

    FILE* outFile = fopen(destPath, "wb");
    if (outFile == NULL) {
        free(buffer);
        free(destPath);
        return NULL;
    }

    size_t written = fwrite(buffer, 1, assetLength, outFile);
    fclose(outFile);
    free(buffer);

    if (written != assetLength) {
        free(destPath);
        return NULL;
    }

    return destPath;
}