#pragma once

#ifdef POLY2TRI_BUILD_DLL
    #define POLY2TRI_API _declspec(dllexport)
#elif !POLY2TRI_BUILD_AS_EXECUTABLE
    #define POLY2TRI_API _declspec(dllimport)
#else
    #define POLY2TRI_API
#endif // POLT2TRI_BUILD_DLL