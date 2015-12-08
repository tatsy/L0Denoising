#ifndef _MT19937AR_H_
#define _MT19937AR_H_

#if defined(WIN32)
#ifdef _DLL_EXPORT
#define _DLL_TYPE __declspec(dllexport)
#else
#define _DLL_TYPE __declspec(dllimport)
#endif
#else
#define _DLL_TYPE
#endif

/* initializes mt[N] with a seed */
_DLL_TYPE void init_genrand(unsigned long s);

/* initialize by an array with array-length */
/* init_key is the array for initializing keys */
/* key_length is its length */
/* slight change for C++, 2004/2/26 */
_DLL_TYPE void init_by_array(unsigned long init_key[], int key_length);

/* generates a random number on [0,0xffffffff]-interval */
_DLL_TYPE unsigned long genrand_int32(void);

/* generates a random number on [0,0x7fffffff]-interval */
_DLL_TYPE long genrand_int31(void);

/* These real versions are due to Isaku Wada, 2002/01/09 added */
/* generates a random number on [0,1]-real-interval */
_DLL_TYPE double genrand_real1(void);

/* generates a random number on [0,1)-real-interval */
_DLL_TYPE double genrand_real2(void);

/* generates a random number on (0,1)-real-interval */
_DLL_TYPE double genrand_real3(void);

/* generates a random number on [0,1) with 53-bit resolution*/
_DLL_TYPE double genrand_res53(void);

#endif
