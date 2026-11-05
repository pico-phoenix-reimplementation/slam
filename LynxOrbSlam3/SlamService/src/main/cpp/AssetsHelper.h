#pragma once

#include <android/asset_manager.h>
#include <string>

class AssetsHelper {
public:
    AssetsHelper(const std::string& internalDataPath, AAssetManager* aAssetManager);
    ~AssetsHelper() = default;

    char* generateDataFilePath(const char* fileName); //Generate the data file path (no file operation is performed)
    char* copyAssetToInternalStorage(const char* assetName); //Copy the asset to the internal storage

private:
    const std::string internalDataPath; //Internal data path
    AAssetManager* aAssetManager; //Asset manager
};
