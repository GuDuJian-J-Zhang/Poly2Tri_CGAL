#pragma once

#ifdef POLY2TRI_BUILD_DLL
    #define POLY2TRI_API _declspec(dllexport)
#else
    #define POLY2TRI_API _declspec(dllimport)
#endif // POLT2TRI_BUILD_DLL