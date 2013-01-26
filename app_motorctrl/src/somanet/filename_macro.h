#pragma once

#define STRFY_HELPER(S) #S
#define STRFY(S) STRFY_HELPER(S)

#define CONCAT3_HELPER(A,B,C) A ## B ## C
#define CONCAT3(A,B,C) CONCAT3_HELPER(A,B,C)
