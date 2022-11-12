#define GEOGRAM_PSM
#ifndef GEO_STATIC_LIBS
#define GEO_DYNAMIC_LIBS
#endif
/*
 *  Copyright (c) 2000-2022 Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact: Bruno Levy
 *
 *     https://www.inria.fr/fr/bruno-levy
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */


/*
 *  This file is a PSM (pluggable software module)
 *   generated from the distribution of Geogram.
 *
 *  See Geogram documentation on:
 *   http://alice.loria.fr/software/geogram/doc/html/index.html
 *
 *  See documentation of the functions bundled in this PSM on:
 *   http://alice.loria.fr/software/geogram/doc/html/multi__precision_8h.html
 */



/******* extracted from ../api/defs.h *******/

#ifndef GEOGRAM_API_DEFS
#define GEOGRAM_API_DEFS


/*
 * Deactivate warnings about documentation
 * We do that, because CLANG's doxygen parser does not know
 * some doxygen commands that we use (retval, copydoc) and
 * generates many warnings for them...
 */
#if defined(__clang__)
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wdocumentation-unknown-command" 
#endif


#if defined(GEO_DYNAMIC_LIBS)
   #if defined(_MSC_VER)
      #define GEO_IMPORT __declspec(dllimport) 
      #define GEO_EXPORT __declspec(dllexport) 
   #elif defined(__GNUC__)
      #define GEO_IMPORT  
      #define GEO_EXPORT __attribute__ ((visibility("default")))
   #else
      #define GEO_IMPORT
      #define GEO_EXPORT
   #endif
#else
   #define GEO_IMPORT
   #define GEO_EXPORT
#endif

#ifdef geogram_EXPORTS
#define GEOGRAM_API GEO_EXPORT
#else
#define GEOGRAM_API GEO_IMPORT
#endif


#define NO_GEOGRAM_API

typedef int GeoMesh;

typedef unsigned char geo_coord_index_t;

/* 
 * If GARGANTUA is defined, then geogram is compiled 
 * with 64 bit indices. 
 */
#ifdef GARGANTUA

#include <stdint.h>

typedef uint64_t geo_index_t;

typedef int64_t geo_signed_index_t;

#else

typedef unsigned int geo_index_t;

typedef int geo_signed_index_t;

#endif

typedef double geo_coord_t;

typedef int geo_boolean;

enum {
    GEO_FALSE = 0,
    GEO_TRUE = 1
};

#endif


/******* extracted from ../basic/common.h *******/

#ifndef GEOGRAM_BASIC_COMMON
#define GEOGRAM_BASIC_COMMON


// iostream should be included before anything else,
// otherwise 'cin', 'cout' and 'cerr' will be uninitialized.
#include <iostream>



namespace GEO {

    enum {
	GEOGRAM_NO_HANDLER = 0,
	GEOGRAM_INSTALL_HANDLERS = 1
    };
    
    void GEOGRAM_API initialize(int flags = GEOGRAM_INSTALL_HANDLERS);

    void GEOGRAM_API terminate();
}


#if (defined(NDEBUG) || defined(GEOGRAM_PSM)) && !defined(GEOGRAM_PSM_DEBUG)
#undef GEO_DEBUG
#undef GEO_PARANOID
#else
#define GEO_DEBUG
#define GEO_PARANOID
#endif

// =============================== LINUX defines ===========================

#if defined(__ANDROID__)
#define GEO_OS_ANDROID
#endif

#if defined(__linux__)

#define GEO_OS_LINUX
#define GEO_OS_UNIX

#ifndef GEO_OS_ANDROID
#define GEO_OS_X11
#endif

#if defined(_OPENMP)
#  define GEO_OPENMP
#endif

#if defined(__INTEL_COMPILER)
#  define GEO_COMPILER_INTEL
#elif defined(__clang__)
#  define GEO_COMPILER_CLANG
#elif defined(__GNUC__)
#  define GEO_COMPILER_GCC
#else
#  error "Unsupported compiler"
#endif

// The following works on GCC and ICC
#if defined(__x86_64)
#  define GEO_ARCH_64
#else
#  define GEO_ARCH_32
#endif

// =============================== WINDOWS defines =========================

#elif defined(_WIN32) || defined(_WIN64)

#define GEO_OS_WINDOWS

#if defined(_OPENMP)
#  define GEO_OPENMP
#endif

#if defined(_MSC_VER)
#  define GEO_COMPILER_MSVC
#elif defined(__MINGW32__) || defined(__MINGW64__)
#  define GEO_COMPILER_MINGW
#endif

#if defined(_WIN64)
#  define GEO_ARCH_64
#else
#  define GEO_ARCH_32
#endif

// =============================== APPLE defines ===========================

#elif defined(__APPLE__)

#define GEO_OS_APPLE
#define GEO_OS_UNIX

#if defined(_OPENMP)
#  define GEO_OPENMP
#endif

#if defined(__clang__)
#  define GEO_COMPILER_CLANG
#elif defined(__GNUC__)
#  define GEO_COMPILER_GCC
#else
#  error "Unsupported compiler"
#endif

#if defined(__x86_64) || defined(__ppc64__) || defined(__arm64__) || defined(__aarch64__)
#  define GEO_ARCH_64
#else
#  define GEO_ARCH_32
#endif

// =============================== Emscripten defines  ======================

#elif defined(__EMSCRIPTEN__)

#define GEO_OS_UNIX
#define GEO_OS_LINUX
#define GEO_OS_EMSCRIPTEN
#define GEO_ARCH_64
#define GEO_COMPILER_EMSCRIPTEN

// =============================== Unsupported =============================
#else
#error "Unsupported operating system"
#endif

#if defined(GEO_COMPILER_GCC)   || \
    defined(GEO_COMPILER_CLANG) || \
    defined(GEO_COMPILER_MINGW) || \
    defined(GEO_COMPILER_EMSCRIPTEN)
#define GEO_COMPILER_GCC_FAMILY
#endif

#ifdef DOXYGEN_ONLY
// Keep doxygen happy
#define GEO_OS_WINDOWS
#define GEO_OS_APPLE
#define GEO_OS_ANDROID
#define GEO_ARCH_32
#define GEO_COMPILER_INTEL
#define GEO_COMPILER_MSVC
#endif

#define CPP_CONCAT_(A, B) A ## B

#define CPP_CONCAT(A, B) CPP_CONCAT_(A, B)

#if defined(GOMGEN)
#define GEO_NORETURN
#elif defined(GEO_COMPILER_GCC_FAMILY) || \
      defined(GEO_COMPILER_INTEL) 
#define GEO_NORETURN __attribute__((noreturn))
#else
#define GEO_NORETURN
#endif

#if defined(GOMGEN)
#define GEO_NORETURN_DECL 
#elif defined(GEO_COMPILER_MSVC)
#define GEO_NORETURN_DECL __declspec(noreturn)
#else
#define GEO_NORETURN_DECL 
#endif

#if defined(GEO_COMPILER_CLANG) || defined(GEO_COMPILER_EMSCRIPTEN)
#if __has_feature(cxx_noexcept)
#define GEO_NOEXCEPT noexcept
#endif
#endif

// For Graphite GOM generator (swig is confused by throw() specifier) 
#ifdef GOMGEN 
#define GEO_NOEXCEPT
#endif

#ifndef GEO_NOEXCEPT
#define GEO_NOEXCEPT throw()
#endif

#define FOR(I,UPPERBND) for(index_t I = 0; I<index_t(UPPERBND); ++I)

// Silence warnings for alloca()
// We use it at different places to allocate objects on the stack
// (for instance, in multi-precision predicates).
#ifdef GEO_COMPILER_CLANG
#pragma GCC diagnostic ignored "-Walloca"
#endif

#endif


/******* extracted from ../basic/argused.h *******/

#ifndef GEOGRAM_BASIC_ARGUSED
#define GEOGRAM_BASIC_ARGUSED



namespace GEO {

    template <class T>
    inline void geo_argused(const T&) {
    }
}

#endif


/******* extracted from ../basic/numeric.h *******/

#ifndef GEOGRAM_BASIC_NUMERIC
#define GEOGRAM_BASIC_NUMERIC

#include <cmath>
#include <float.h>
#include <limits.h>
#include <algorithm> // for std::min / std::max

// Visual C++ ver. < 2010 does not have C99 stdint.h,
// using a fallback portable one.
#if defined(GEO_OS_WINDOWS) && (_MSC_VER < 1600)
#else
#include <stdint.h>
#endif

#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace GEO {

    namespace Numeric {

        
        typedef void* pointer;

        
        typedef int8_t int8;

        
        typedef int16_t int16;

        
        typedef int32_t int32;

        
        typedef int64_t int64;

        
        typedef uint8_t uint8;

        
        typedef uint16_t uint16;

        
        typedef uint32_t uint32;

        
        typedef uint64_t uint64;

        
        typedef float float32;

        
        typedef double float64;

        inline float32 max_float32() {
            return std::numeric_limits<float32>::max();
        }

        inline float32 min_float32() {
            // Note: numeric_limits<>::min() is not
            // what we want (it returns the smallest
            // positive non-denormal).
            return -max_float32();
        }

        inline float64 max_float64() {
            return std::numeric_limits<float64>::max();
        }

        inline float64 min_float64() {
            // Note: numeric_limits<>::min() is not
            // what we want (it returns the smallest
            // positive non-denormal).
            return -max_float64();
        }

        bool GEOGRAM_API is_nan(float32 x);

        bool GEOGRAM_API is_nan(float64 x);

        void GEOGRAM_API random_reset();

        int32 GEOGRAM_API random_int32();

        float32 GEOGRAM_API random_float32();

        float64 GEOGRAM_API random_float64();

        template <class T, bool is_numeric>
        struct LimitsHelper : std::numeric_limits<T> {
        };

        template <class T>
        struct LimitsHelper<T, true> : std::numeric_limits<T> {
            
            static const size_t size = sizeof(T);
            
            static const size_t numbits = 8 * sizeof(T);
        };

        template <class T>
        struct Limits : 
            LimitsHelper<T, std::numeric_limits<T>::is_specialized> {
        };
    }

    

    enum Sign {
        
        NEGATIVE = -1,
        
        ZERO = 0,
        
        POSITIVE = 1
    };

    template <class T>
    inline Sign geo_sgn(const T& x) {
        return (x > 0) ? POSITIVE : (
            (x < 0) ? NEGATIVE : ZERO
        );
    }

    template <class T>
    inline T geo_sqr(T x) {
        return x * x;
    }

    template <class T>
    inline void geo_clamp(T& x, T min, T max) {
        if(x < min) {
            x = min;
        } else if(x > max) {
            x = max;
        }
    }

    typedef geo_index_t index_t;

    inline index_t max_index_t() {
        return std::numeric_limits<index_t>::max();
    }

    typedef geo_signed_index_t signed_index_t;

    inline signed_index_t max_signed_index_t() {
        return std::numeric_limits<signed_index_t>::max();
    }

    inline signed_index_t min_signed_index_t() {
        return std::numeric_limits<signed_index_t>::min();
    }

    typedef geo_coord_index_t coord_index_t;

    inline double round(double x) {
	return ((x - floor(x)) > 0.5 ? ceil(x) : floor(x));
    }
}

#endif


/******* extracted from ../basic/memory.h *******/

#ifndef GEOGRAM_BASIC_MEMORY
#define GEOGRAM_BASIC_MEMORY

#include <vector>
#include <string.h>
#include <stdlib.h>

#ifdef GEO_OS_WINDOWS

#include <windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#else

#include <unistd.h>

#endif


namespace GEO {

    namespace Memory {
        
        typedef unsigned char byte;

        
        typedef unsigned char word8;

        
        typedef unsigned short word16;

        
        typedef unsigned int word32;

        
        typedef byte* pointer;

	
	typedef void (*function_pointer)();
	
        inline void clear(void* addr, size_t size) {
            ::memset(addr, 0, size);
        }

        inline void copy(void* to, const void* from, size_t size) {
            ::memcpy(to, from, size);
        }

	inline pointer function_pointer_to_generic_pointer(function_pointer fptr) {
	    // I know this is ugly, but I did not find a simpler warning-free
	    // way that is portable between all compilers.
	    pointer result = nullptr;
	    ::memcpy(&result, &fptr, sizeof(pointer));
	    return result;
	}

	inline function_pointer generic_pointer_to_function_pointer(pointer ptr) {
	    // I know this is ugly, but I did not find a simpler warning-free
	    // way that is portable between all compilers.
	    function_pointer result = nullptr;
	    ::memcpy(&result, &ptr, sizeof(pointer));
	    return result;
	}

	inline function_pointer generic_pointer_to_function_pointer(void* ptr) {
	    // I know this is ugly, but I did not find a simpler warning-free
	    // way that is portable between all compilers.
	    function_pointer result = nullptr;
	    ::memcpy(&result, &ptr, sizeof(pointer));
	    return result;
	}
	
#define GEO_MEMORY_ALIGNMENT 64

        template <int DIM>
        struct PointAlignment {
            static const size_t value = 1;
        };

        template <>
        struct PointAlignment<2> {
            static const size_t value = 16;
        };

        template <>
        struct PointAlignment<3> {
            static const size_t value = 8;
        };

        template <>
        struct PointAlignment<4> {
            static const size_t value = 32;
        };

        template <>
        struct PointAlignment<6> {
            static const size_t value = 16;
        };

        template <>
        struct PointAlignment<8> {
            static const size_t value = 64;
        };

#define geo_dim_alignment(dim) GEO::Memory::PointAlignment<dim>::value

        inline void* aligned_malloc(
            size_t size, size_t alignment = GEO_MEMORY_ALIGNMENT
        ) {
#if   defined(GEO_OS_ANDROID)
            // Alignment not supported under Android.
            geo_argused(alignment);
            return malloc(size);
#elif defined(GEO_COMPILER_INTEL)
            return _mm_malloc(size, alignment);
#elif defined(GEO_COMPILER_GCC) || defined(GEO_COMPILER_CLANG)
            void* result;
            return posix_memalign(&result, alignment, size) == 0
                   ? result : nullptr;
#elif defined(GEO_COMPILER_MSVC)
            return _aligned_malloc(size, alignment);
#else
            geo_argused(alignment);
            return malloc(size);
#endif
        }

        inline void aligned_free(void* p) {
#if   defined(GEO_OS_ANDROID)
            // Alignment not supported under Android.
            free(p);
#elif defined(GEO_COMPILER_INTEL)
            _mm_free(p);
#elif defined(GEO_COMPILER_GCC_FAMILY) 
            free(p);
#elif defined(GEO_COMPILER_MSVC)
            _aligned_free(p);
#else
            free(p);
#endif
        }

#if   defined(GEO_OS_ANDROID)
#define geo_decl_aligned(var) var
#elif defined(GEO_COMPILER_INTEL)
#define geo_decl_aligned(var) __declspec(aligned(GEO_MEMORY_ALIGNMENT)) var
#elif defined(GEO_COMPILER_GCC_FAMILY)
#define geo_decl_aligned(var) var __attribute__((aligned(GEO_MEMORY_ALIGNMENT)))
#elif defined(GEO_COMPILER_MSVC)
#define geo_decl_aligned(var) __declspec(align(GEO_MEMORY_ALIGNMENT)) var
#elif defined(GEO_COMPILER_EMSCRIPTEN)
#define geo_decl_aligned(var) var        
#endif

#if   defined(GEO_OS_ANDROID)
#define geo_assume_aligned(var, alignment)
#elif defined(GEO_COMPILER_INTEL)
#define geo_assume_aligned(var, alignment) \
    __assume_aligned(var, alignment)
#elif defined(GEO_COMPILER_CLANG)
#define geo_assume_aligned(var, alignment)
        // GCC __builtin_assume_aligned is not yet supported by clang-3.3
#elif defined(GEO_COMPILER_GCC)
#if __GNUC__ >= 4 && __GNUC_MINOR__ >= 7
#define geo_assume_aligned(var, alignment) \
        *(void**) (&var) = __builtin_assume_aligned(var, alignment)
        // the GCC way of specifying that a pointer is aligned returns
        // the aligned pointer (I can't figure out why). It needs to be
        // affected otherwise it is not taken into account (verified by
        // looking at the output of gcc -S)
#else
#define geo_assume_aligned(var, alignment)        
#endif        
#elif defined(GEO_COMPILER_MSVC) 
#define geo_assume_aligned(var, alignment)
        // TODO: I do not know how to do that with MSVC
#elif defined(GEO_COMPILER_EMSCRIPTEN)        
#define geo_assume_aligned(var, alignment)
#elif defined(GEO_COMPILER_MINGW)        
#define geo_assume_aligned(var, alignment)
#endif

#if   defined(GEO_COMPILER_INTEL)
#define geo_restrict __restrict
#elif defined(GEO_COMPILER_GCC_FAMILY)
#define geo_restrict __restrict__
#elif defined(GEO_COMPILER_MSVC)
#define geo_restrict __restrict
#elif defined(GEO_COMPILER_EMSCRIPTEN)
#define geo_restrict 
#endif

        inline bool is_aligned(
            void* p, size_t alignment = GEO_MEMORY_ALIGNMENT
        ) {
            return (reinterpret_cast<size_t>(p) & (alignment - 1)) == 0;
        }

        inline void* align(void* p) {
            size_t offset = (
                GEO_MEMORY_ALIGNMENT -
                (reinterpret_cast<size_t>(p) & (GEO_MEMORY_ALIGNMENT - 1))
            ) & (GEO_MEMORY_ALIGNMENT - 1);
            return reinterpret_cast<char*>(p) + offset;
        }

#define geo_aligned_alloca(size) \
    GEO::Memory::align(alloca(size + GEO_MEMORY_ALIGNMENT - 1))

        template <class T, int ALIGN = GEO_MEMORY_ALIGNMENT>
        class aligned_allocator {
        public:
            
            typedef T value_type;

            
            typedef T* pointer;

            
            typedef T& reference;

            
            typedef const T* const_pointer;

            
            typedef const T& const_reference;

            
            typedef ::std::size_t size_type;

            
            typedef ::std::ptrdiff_t difference_type;

            template <class U>
            struct rebind {
                
                typedef aligned_allocator<U> other;
            };

            pointer address(reference x) {
                return &x;
            }

            const_pointer address(const_reference x) {
                return &x;
            }

            pointer allocate(
                size_type nb_elt, const void* hint = nullptr
            ) {
                geo_argused(hint);
                pointer result = static_cast<pointer>(
                    aligned_malloc(sizeof(T) * nb_elt, ALIGN)
                );
                return result;
            }

            void deallocate(pointer p, size_type nb_elt) {
                geo_argused(nb_elt);
                aligned_free(p);
            }

            size_type max_size() const {
                ::std::allocator<char> a;
                return std::allocator_traits<decltype(a)>::max_size(a) / sizeof(T);
            }

            void construct(pointer p, const_reference val) {
                new (static_cast<void*>(p))value_type(val);
            }

            void destroy(pointer p) {
                p->~value_type();
#ifdef GEO_COMPILER_MSVC
                (void) p; // to avoid a "unreferenced variable" warning
#endif
            }

            template <class T2, int A2> operator aligned_allocator<T2, A2>() {
                return aligned_allocator<T2,A2>();
            }
        };

        template <typename T1, int A1, typename T2, int A2>
        inline bool operator== (
            const aligned_allocator<T1, A1>&, const aligned_allocator<T2, A2>&
        ) {
            return true;
        }

        template <typename T1, int A1, typename T2, int A2>
        inline bool operator!= (
            const aligned_allocator<T1, A1>&, const aligned_allocator<T2, A2>&
        ) {
            return false;
        }
    }

    

    template <class T>
    class vector : public ::std::vector<T, Memory::aligned_allocator<T> > {
        typedef ::std::vector<T, Memory::aligned_allocator<T> > baseclass;

    public:
        vector() :
            baseclass() {
        }

        explicit vector(index_t size) :
            baseclass(size) {
        }

        explicit vector(index_t size, const T& val) :
            baseclass(size, val) {
        }

        index_t size() const {
            //   casts baseclass::size() from size_t (64 bits)
            //   to index_t (32 bits), because all
            //   indices in Vorpaline are supposed to fit in 32 bits (index_t).
            // TODO: geo_debug_assert(baseclass::size() < max index_t)
            return index_t(baseclass::size());
        }

        T& operator[] (index_t i) {
            geo_debug_assert(i < size());
            return baseclass::operator[] (i);
        }

        const T& operator[] (index_t i) const {
            geo_debug_assert(i < size());
            return baseclass::operator[] (i);
        }

        T& operator[] (signed_index_t i) {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }

        const T& operator[] (signed_index_t i) const {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }


#ifdef GARGANTUA // If compiled with 64 bits index_t

        T& operator[] (int i) {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }

        const T& operator[] (int i) const {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }

        T& operator[] (unsigned int i) {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }

        const T& operator[] (unsigned int i) const {
            geo_debug_assert(i >= 0 && index_t(i) < size());
            return baseclass::operator[] (index_t(i));
        }
#endif	
	
        T* data() {
            return size() == 0 ? nullptr : &(*this)[0];
        }

        const T* data() const {
            return size() == 0 ? nullptr : &(*this)[0];
        }

    };

    template <>
    class vector<bool> : public ::std::vector<bool> {
        typedef ::std::vector<bool> baseclass;

    public:
        
        vector() :
            baseclass() {
        }

        
        explicit vector(index_t size) :
            baseclass(size) {
        }

        
        explicit vector(index_t size, bool val) :
            baseclass(size, val) {
        }

        
        index_t size() const {
            //   casts baseclass::size() from size_t (64 bits)
            //   to index_t (32 bits), because all
            //   indices in Vorpaline are supposed to fit in 32 bits (index_t).
            // TODO: geo_debug_assert(baseclass::size() < max index_t)
            return index_t(baseclass::size());
        }

        // TODO: operator[] with bounds checking (more complicated
        // than just returning bool&, check implementation in STL).
    };
}

#endif


/******* extracted from ../basic/atomics.h *******/

#ifndef GEOGRAM_BASIC_ATOMICS
#define GEOGRAM_BASIC_ATOMICS



#ifdef GEO_OS_LINUX
#  if defined(GEO_OS_EMSCRIPTEN) 
#    define GEO_USE_DUMMY_ATOMICS
#  elif defined(GEO_OS_RASPBERRY)
#    define GEO_USE_ARM32_ATOMICS
#  elif defined(GEO_OS_ANDROID)
#    define GEO_USE_ANDROID_ATOMICS
#  else
#    define GEO_USE_X86_ATOMICS
#  endif
#endif

#if defined(GEO_USE_DUMMY_ATOMICS)

inline void geo_pause() {
}

inline char atomic_bittestandset_x86(volatile unsigned int*, unsigned int) {
    return 0;
}

inline char atomic_bittestandreset_x86(volatile unsigned int*, unsigned int) {
    return 0;
}

#elif defined(GEO_USE_ANDROID_ATOMICS)


typedef GEO::Numeric::uint32 android_mutex_t;

inline void lock_mutex_android(volatile android_mutex_t* lock) {
    while(__sync_lock_test_and_set(lock, 1) != 0);
}

inline void unlock_mutex_android(volatile android_mutex_t* lock) {
    __sync_lock_release(lock);
}

inline unsigned int atomic_bitset_android(volatile unsigned int* ptr, unsigned int bit) {
    return __sync_fetch_and_or(ptr, 1u << bit) & (1u << bit);
}

inline unsigned int atomic_bitreset_android(volatile unsigned int* ptr, unsigned int bit) {
    return __sync_fetch_and_and(ptr, ~(1u << bit)) & (1u << bit);
}

inline void memory_barrier_android() {
    // Full memory barrier.
    __sync_synchronize();
}

inline void wait_for_event_android() {
    /* TODO */    
}

inline void send_event_android() {
    /* TODO */    
}

#elif defined(GEO_USE_ARM32_ATOMICS)


typedef GEO::Numeric::uint32 arm32_mutex_t;

inline void lock_mutex_arm32(volatile arm32_mutex_t* lock) {
    arm_mutex_t tmp;
    __asm__ __volatile__ (
        "1:     ldrex   %0, [%1]     \n" // read lock
        "       cmp     %0, #0       \n" // check if zero
        "       wfene                \n" // wait for event if non-zero
        "       strexeq %0, %2, [%1] \n" // attempt to store new value
        "       cmpeq   %0, #0       \n" // test if store succeeded
        "       bne     1b           \n" // retry if not
        "       dmb                  \n" // memory barrier
        : "=&r" (tmp)
        : "r" (lock), "r" (1)
        : "cc", "memory");
}

inline void unlock_mutex_arm32(volatile arm32_mutex_t* lock) {
    __asm__ __volatile__ (
        "       dmb              \n" // ensure all previous access are observed
        "       str     %1, [%0] \n" // clear the lock
        "       dsb              \n" // ensure completion of clear lock ...
        "       sev              \n" // ... before sending the event
        :
        : "r" (lock), "r" (0)
        : "cc", "memory");
}

inline unsigned int atomic_bitset_arm32(volatile unsigned int* ptr, unsigned int bit) {
    unsigned int tmp;
    unsigned int result;
    unsigned int OK;
    __asm__ __volatile__ (
        "1:     ldrex   %1, [%5]           \n" // result = *ptr
        "       orr     %0, %1, %6, LSL %4 \n" // tmp = result OR (1 << bit)
        "       strex   %2, %0, [%5]       \n" // *ptr = tmp, status in OK
        "       teq     %2, #0             \n" // if !OK then
        "       bne     1b                 \n" //    goto 1:
        "       and     %1, %1, %6, LSL %4 \n" // result = result AND (1 << bit)
        : "=&r" (tmp), "=&r" (result), "=&r" (OK), "+m" (*ptr)
        : "r" (bit), "r" (ptr), "r" (1)
        : "cc"
    );
    return result;
}

inline unsigned int atomic_bitreset_arm32(volatile unsigned int* ptr, unsigned int bit) {
    unsigned int tmp;
    unsigned int result;
    unsigned int OK;
    __asm__ __volatile__ (
        "1:     ldrex   %1, [%5]           \n" // result = *ptr
        "       bic     %0, %1, %6, LSL %4 \n" // tmp = result AND NOT(1 << bit)
        "       strex   %2, %0, [%5]       \n" // *ptr = tmp, status in OK
        "       teq     %2, #0             \n" // if !OK then
        "       bne     1b                 \n" //    goto 1:
        "       and     %1, %1, %6, LSL %4 \n" // result = result AND (1 << bit)
        : "=&r" (tmp), "=&r" (result), "=&r" (OK), "+m" (*ptr)
        : "r" (bit), "r" (ptr), "r" (1)
        : "cc"
    );
    return result;
}

inline void memory_barrier_arm32() {
    __asm__ __volatile__ (
        "dmb \n"
        : : : "memory"
    );
}

inline void wait_for_event_arm32() {
    __asm__ __volatile__ (
        "wfe \n"
        : : : 
    );
}

inline void send_event_arm32() {
    __asm__ __volatile__ (
        "dsb \n" // ensure completion of store operations
        "sev \n"
        : : : 
    );
}

#elif defined(GEO_USE_X86_ATOMICS)

#  define GEO_USE_X86_PAUSE

#  ifdef GEO_USE_X86_PAUSE

inline void geo_pause() {
    __asm__ __volatile__ (
        "pause;\n"
    );
}

#  else
#    ifdef __ICC
#      define geo_pause _mm_pause
#    else
#      define geo_pause __builtin_ia32_pause
#    endif

#  endif

inline char atomic_bittestandset_x86(volatile unsigned int* ptr, unsigned int bit) {
    char out;
#if defined(__x86_64)
    __asm__ __volatile__ (
        "lock; bts %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then set bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=r" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#else
    __asm__ __volatile__ (
        "lock; bts %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then set bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=q" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#endif
    return out;
}

inline char atomic_bittestandreset_x86(volatile unsigned int* ptr, unsigned int bit) {
    char out;
#if defined(__x86_64)
    __asm__ __volatile__ (
        "lock; btr %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then reset bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=r" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#else
    __asm__ __volatile__ (
        "lock; btr %2,%1\n"  // set carry flag if bit %2 (bit) of %1 (ptr) is set
                             //   then reset bit %2 of %1
        "sbb %0,%0\n"        // set %0 (out) if carry flag is set
        : "=q" (out), "=m" (*ptr)
        : "Ir" (bit)
        : "memory"
    );
#endif
    return out;
}

#elif defined(GEO_OS_APPLE)

#include <libkern/OSAtomic.h>

#elif defined(GEO_OS_WINDOWS)

#include <windows.h>
#include <intrin.h>
#pragma intrinsic(_InterlockedCompareExchange8)
#pragma intrinsic(_InterlockedCompareExchange16)
#pragma intrinsic(_InterlockedCompareExchange)
#pragma intrinsic(_interlockedbittestandset)
#pragma intrinsic(_interlockedbittestandreset)
#pragma intrinsic(_ReadBarrier)
#pragma intrinsic(_WriteBarrier)
#pragma intrinsic(_ReadWriteBarrier)

#  ifdef GEO_COMPILER_MINGW
inline void geo_pause() {
}
#  endif

#endif // GEO_OS_WINDOWS

#endif


/******* extracted from ../basic/thread_sync.h *******/

#ifndef GEOGRAM_BASIC_THREAD_SYNC
#define GEOGRAM_BASIC_THREAD_SYNC

#include <vector>

#ifdef GEO_OS_APPLE
# define GEO_USE_DEFAULT_SPINLOCK_ARRAY
# include <AvailabilityMacros.h>
# if defined(MAC_OS_X_VERSION_10_12) && MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_12
#   define GEO_APPLE_HAS_UNFAIR_LOCK 1
#   include <os/lock.h>
# endif
# if defined(__IPHONE_OS_VERSION_MIN_REQUIRED) && __IPHONE_OS_VERSION_MIN_REQUIRED >= __IPHONE_10_0
#   define GEO_APPLE_HAS_UNFAIR_LOCK 1
#   include <os/lock.h>
# endif
#endif

#ifdef geo_debug_assert
#define geo_thread_sync_assert(x) geo_debug_assert(x)
#else
#define geo_thread_sync_assert(x) 
#endif


namespace GEO {

    namespace Process {

#if defined(GEO_OS_RASPBERRY)

        
        typedef arm32_mutex_t spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(spinlock& x) {
            lock_mutex_arm32(&x);
        }

        inline void release_spinlock(spinlock& x) {
            unlock_mutex_arm32(&x);
        }
	
#elif defined(GEO_OS_ANDROID)

        
        typedef android_mutex_t spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(spinlock& x) {
            lock_mutex_android(&x);
        }

        inline void release_spinlock(spinlock& x) {
            unlock_mutex_android(&x);
        }

#elif defined(GEO_OS_LINUX) || defined(GEO_COMPILER_MINGW)

        
        typedef unsigned char spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(volatile spinlock& x) {
            while(__sync_lock_test_and_set(&x, 1) == 1) {
                // Intel recommends to have a PAUSE asm instruction
                // in the spinlock loop.
                geo_pause();
            }
        }

        inline void release_spinlock(volatile spinlock& x) {
            // Note: Since on intel processor, memory writes
            // (of data types <= bus size) are atomic, we could
            // simply write 'x=0' instead, but this would
            // lack the 'memory barrier with release semantics'
            // required to avoid compiler and/or processor
            // reordering (important for relaxed memory
            // models such as Itanium processors).
            __sync_lock_release(&x);
        }

#elif defined(GEO_OS_APPLE)

#if defined(GEO_APPLE_HAS_UNFAIR_LOCK)
        
        typedef os_unfair_lock spinlock;
        
        
#       define GEOGRAM_SPINLOCK_INIT OS_UNFAIR_LOCK_INIT
        //inline void init_spinlock(spinlock & s) { s = OS_UNFAIR_LOCK_INIT; }
        //inline bool try_acquire_spinlock (spinlock & s) { return os_unfair_lock_trylock(&s); }
        inline void acquire_spinlock    (spinlock & s) { os_unfair_lock_lock(&s); }
        inline void release_spinlock  (spinlock & s) { os_unfair_lock_unlock(&s); }
#else
        
        typedef OSSpinLock spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT OS_SPINLOCK_INIT
        inline void acquire_spinlock(volatile spinlock& x) {
            OSSpinLockLock(&x);
        }

        inline void release_spinlock(volatile spinlock& x) {
            OSSpinLockUnlock(&x);
        }
#endif // __MAC_10_12

#elif defined(GEO_OS_WINDOWS) && !defined(GEO_COMPILER_MINGW)

        
        typedef short spinlock;

        
#       define GEOGRAM_SPINLOCK_INIT 0
        inline void acquire_spinlock(volatile spinlock& x) {
            while(_InterlockedCompareExchange16(&x, 1, 0) == 1) {
                // Intel recommends to have a PAUSE asm instruction
                // in the spinlock loop. Under MSVC/Windows,
                // YieldProcessor() is a macro that calls the
                // (undocumented) _mm_pause() intrinsic function
                // that generates a PAUSE opcode.
                YieldProcessor();
            }
            // We do not need _ReadBarrier() here since
            // _InterlockedCompareExchange16
            // "acts as a full barrier in VC2005" according to the doc
        }

        inline void release_spinlock(volatile spinlock& x) {
            _WriteBarrier();   // prevents compiler reordering
            x = 0;
        }

#endif

#if defined(GEO_USE_DEFAULT_SPINLOCK_ARRAY)

        // TODO: implement memory-efficient version for
        // MacOSX MacOSX does have atomic bit
        // manipulation routines (OSAtomicTestAndSet()),
        // and also  has OSAtomicAnd32OrigBarrier() and
        // OSAtomicOr32OrigBarrier() functions that can
        // be used instead (Orig for 'return previous value'
        // and Barrier for ('include a memory barrier').
        // Android needs additional routines in atomics.h/atomics.cpp

        class SpinLockArray {
        public:
            SpinLockArray() {
            }

            SpinLockArray(index_t size_in) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                spinlocks_.assign(size_in, GEOGRAM_SPINLOCK_INIT);
            }

            void clear() {
                spinlocks_.clear();
            }

            index_t size() const {
                return index_t(spinlocks_.size());
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                GEO::Process::acquire_spinlock(spinlocks_[i]);
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                GEO::Process::release_spinlock(spinlocks_[i]);
            }

        private:
            std::vector<spinlock> spinlocks_;
        };

#elif defined(GEO_OS_RASPBERRY) 

        class SpinLockArray {
        public:
            typedef Numeric::uint32 word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                word_t b = word_t(i & 31);
                // Loop while previously stored value has its bit set.
                while((atomic_bitset_arm32(&spinlocks_[w], b)) != 0) {
                    // If somebody else has the lock, sleep.
                    //  It is important to sleep here, else atomic_bitset_xxx()
                    // keeps acquiring the exclusive monitor (even for testing)
                    // and this slows down everything.
                    wait_for_event_arm32();
                }
                memory_barrier_arm32();
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                memory_barrier_android();
                index_t w = i >> 5;
                word_t b = word_t(i & 31);
                atomic_bitreset_arm32(&spinlocks_[w], b);
                //   Now wake up the other threads that started
                // sleeping if they did not manage to acquire
                // the lock.
                send_event_arm32();
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };
	
#elif defined(GEO_OS_ANDROID) 

        class SpinLockArray {
        public:
            typedef Numeric::uint32 word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                word_t b = word_t(i & 31);
                // Loop while previously stored value has its bit set.
                while((atomic_bitset_android(&spinlocks_[w], b)) != 0) {
                    // If somebody else has the lock, sleep.
                    //  It is important to sleep here, else atomic_bitset_xxx()
                    // keeps acquiring the exclusive monitor (even for testing)
                    // and this slows down everything.
                    wait_for_event_android();
                }
                memory_barrier_android();
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                memory_barrier_android();
                index_t w = i >> 5;
                word_t b = word_t(i & 31);
                atomic_bitreset_android(&spinlocks_[w], b);
                //   Now wake up the other threads that started
                // sleeping if they did not manage to acquire
                // the lock.
                send_event_android();
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };

#elif defined(GEO_OS_LINUX) 

        class SpinLockArray {
        public:
            typedef Numeric::uint32 word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                while(atomic_bittestandset_x86(&spinlocks_[w], Numeric::uint32(b))) {
                    // Intel recommends to have a PAUSE asm instruction
                    // in the spinlock loop. It is generated using the
                    // following intrinsic function of GCC.
                    geo_pause();
                }
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                // Note: we need here to use a synchronized bit reset
                // since &= is not atomic.
                atomic_bittestandreset_x86(&spinlocks_[w], Numeric::uint32(b));
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };

#elif defined(GEO_OS_WINDOWS)
        class SpinLockArray {
        public:
            typedef LONG word_t;

            SpinLockArray() : size_(0) {
            }

            SpinLockArray(index_t size_in) : size_(0) {
                resize(size_in);
            }

            void resize(index_t size_in) {
                if(size_ != size_in) {
                    size_ = size_in;
                    index_t nb_words = (size_ >> 5) + 1;
                    spinlocks_.assign(nb_words, 0);
                }
            }

            index_t size() const {
                return size_;
            }

            void clear() {
                spinlocks_.clear();
            }

            void acquire_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                while(_interlockedbittestandset((long *)(&spinlocks_[w]), long(b))) {
                    // Intel recommends to have a PAUSE asm instruction
                    // in the spinlock loop. Under MSVC/Windows,
                    // YieldProcessor() is a macro that calls the
                    // (undocumented) _mm_pause() intrinsic function
                    // that generates a PAUSE opcode.
                    YieldProcessor();
                }
                // We do not need here _ReadBarrier() since
                // _interlockedbittestandset
                // "acts as a full barrier in VC2005" according to the doc
            }

            void release_spinlock(index_t i) {
                geo_thread_sync_assert(i < size());
                index_t w = i >> 5;
                index_t b = i & 31;
                // Note1: we need here to use a synchronized bit reset
                // since |= is not atomic.
                // Note2: We do not need here _WriteBarrier() since
                // _interlockedbittestandreset
                // "acts as a full barrier in VC2005" according to the doc
                _interlockedbittestandreset((long*)(&spinlocks_[w]), long(b));
            }

        private:
            std::vector<word_t> spinlocks_;
            index_t size_;
        };

#else

#error Found no implementation of SpinLockArray

#endif


    }

#ifdef GEO_OS_WINDOWS

    // Emulation of pthread mutexes using Windows API

    typedef CRITICAL_SECTION pthread_mutex_t;
    typedef unsigned int pthread_mutexattr_t;
    
    inline int pthread_mutex_lock(pthread_mutex_t *m) {
        EnterCriticalSection(m);
        return 0;
    }

    inline int pthread_mutex_unlock(pthread_mutex_t *m) {
        LeaveCriticalSection(m);
        return 0;
    }
        
    inline int pthread_mutex_trylock(pthread_mutex_t *m) {
        return TryEnterCriticalSection(m) ? 0 : EBUSY; 
    }

    inline int pthread_mutex_init(pthread_mutex_t *m, pthread_mutexattr_t *a) {
        geo_argused(a);
        InitializeCriticalSection(m);
        return 0;
    }

    inline int pthread_mutex_destroy(pthread_mutex_t *m) {
        DeleteCriticalSection(m);
        return 0;
    }


    // Emulation of pthread condition variables using Windows API

    typedef CONDITION_VARIABLE pthread_cond_t;
    typedef unsigned int pthread_condattr_t;

    inline int pthread_cond_init(pthread_cond_t *c, pthread_condattr_t *a) {
        geo_argused(a);
        InitializeConditionVariable(c);
        return 0;
    }

    inline int pthread_cond_destroy(pthread_cond_t *c) {
        geo_argused(c);
        return 0;
    }

    inline int pthread_cond_broadcast(pthread_cond_t *c) {
        WakeAllConditionVariable(c);
        return 0;
    }

    inline int pthread_cond_wait(pthread_cond_t *c, pthread_mutex_t *m) {
        SleepConditionVariableCS(c, m, INFINITE);
        return 0;
    }
   
#endif    
    
}

#endif


/******* extracted from ../basic/psm.h *******/

#ifndef GEOGRAM_BASIC_PSM
#define GEOGRAM_BASIC_PSM


#include <assert.h>
#include <iostream>
#include <string>

#ifndef GEOGRAM_PSM
#define GEOGRAM_PSM
#endif

#ifndef GEOGRAM_BASIC_ASSERT

#define geo_assert(x) assert(x)
#define geo_range_assert(x, min_val, max_val) \
    assert((x) >= (min_val) && (x) <= (max_val))
#define geo_assert_not_reached assert(0)

#ifdef GEO_DEBUG
#define geo_debug_assert(x) assert(x)
#define geo_debug_range_assert(x, min_val, max_val) \
    assert((x) >= (min_val) && (x) <= (max_val))
#else
#define geo_debug_assert(x) 
#define geo_debug_range_assert(x, min_val, max_val)
#endif

#ifdef GEO_PARANOID
#define geo_parano_assert(x) geo_assert(x)
#define geo_parano_range_assert(x, min_val, max_val) \
    geo_range_assert(x, min_val, max_val)
#else
#define geo_parano_assert(x)
#define geo_parano_range_assert(x, min_val, max_val)
#endif

#endif

#ifndef geo_cite
#define geo_cite(x)
#endif

#ifndef geo_cite_with_info
#define geo_cite_with_info(x,y)
#endif

#ifndef GEOGRAM_BASIC_LOGGER

namespace GEO {
    namespace Logger {
        inline std::ostream& out(const std::string& name) {
            return std::cout << " [" << name << "]";
        }

        inline std::ostream& err(const std::string& name) {
            return std::cerr << "E[" << name << "]";
        }

        inline std::ostream& warn(const std::string& name) {
            return std::cerr << "W[" << name << "]";
        }
    }
    
}

#endif

#ifndef FPG_UNCERTAIN_VALUE
#define FPG_UNCERTAIN_VALUE 0
#endif



#ifndef GEOGRAM_BASIC_THREAD_SYNC
#define GEOGRAM_SPINLOCK_INIT 0

namespace GEO {
    namespace Process {
    
        typedef int spinlock;
        
        inline void acquire_spinlock(spinlock& x) {
            // Not implemented yet for PSMs
            geo_argused(x);
            geo_assert_not_reached;
        }
    
        inline void release_spinlock(spinlock& x) {
            // Not implemented yet for PSMs
            geo_argused(x); 
            geo_assert_not_reached;       
        }
    }
}
#endif

#define GEOGRAM_WITH_PDEL

#endif

/******* extracted from multi_precision.h *******/

#ifndef GEOGRAM_NUMERICS_MULTI_PRECISION
#define GEOGRAM_NUMERICS_MULTI_PRECISION

#include <iostream>
#include <new>


namespace GEO {

    extern double expansion_splitter_;
    extern double expansion_epsilon_;

    inline void two_sum(double a, double b, double& x, double& y) {
        x = a + b;
        double bvirt = x - a;
        double avirt = x - bvirt;
        double bround = b - bvirt;
        double around = a - avirt;
        y = around + bround;
    }

    inline void two_diff(double a, double b, double& x, double& y) {
        x = a - b;
        double bvirt = a - x;
        double avirt = x + bvirt;
        double bround = bvirt - b;
        double around = a - avirt;
        y = around + bround;
    }

    inline void split(double a, double& ahi, double& alo) {
        double c = expansion_splitter_ * a;
        double abig = c - a;
        ahi = c - abig;
        alo = a - ahi;
    }

    inline void two_product(double a, double b, double& x, double& y) {
#ifdef FP_FAST_FMA
        // If the target processor supports the FMA (Fused Multiply Add)
        // instruction, then the product of two doubles into a length-2
        // expansion can be implemented as follows. Thanks to Marc Glisse
        // for the information.
        // Note: under gcc, automatic generations of fma() for a*b+c needs
        // to be deactivated, using -ffp-contract=off, else it may break
        // other functions such as fast_expansion_sum_zeroelim().
        x = a*b;
        y = fma(a,b,-x);
#else
        x = a * b;
        double ahi, alo;
        split(a, ahi, alo);
        double bhi, blo;
        split(b, bhi, blo);
        double err1 = x - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        y = (alo * blo) - err3;
#endif
    }

    inline void square(double a, double& x, double& y) {
#ifdef FP_FAST_FMA
        // If the target processor supports the FMA (Fused Multiply Add)
        // instruction, then the product of two doubles into a length-2
        // expansion can be implemented as follows. Thanks to Marc Glisse
        // for the information.
        // Note: under gcc, automatic generations of fma() for a*b+c needs
        // to be deactivated, using -ffp-contract=off, else it may break
        // other functions such as fast_expansion_sum_zeroelim().
        x = a*a;
        y = fma(a,a,-x);
#else
        x = a * a;
        double ahi, alo;
        split(a, ahi, alo);
        double err1 = x - (ahi * ahi);
        double err3 = err1 - ((ahi + ahi) * alo);
        y = (alo * alo) - err3;
#endif
    }

    

    class GEOGRAM_API expansion {
    public:
        index_t length() const {
            return length_;
        }

        index_t capacity() const {
            return capacity_;
        }

        void set_length(index_t new_length) {
            geo_debug_assert(new_length <= capacity());
            length_ = new_length;
        }

        const double& operator[] (index_t i) const {
            // Note: we allocate capacity+1 storage
            // systematically, since basic functions
            // may access one additional value (without
            // using it)
            geo_debug_assert(i <= capacity_);
            return x_[i];
        }

        double& operator[] (index_t i) {
            // Note: we allocate capacity+1 storage
            // systematically, since basic functions
            // may access one additional value (without
            // using it)
            geo_debug_assert(i <= capacity_);
            return x_[i];
        }

        double* data() {
            return x_;
        }

        const double* data() const {
            return x_;
        }

        static size_t bytes(index_t capa) {
            // --> 2*sizeof(double) because x_ is declared of size [2]
            // to avoid compiler's warning.
            // --> capa+1 to have an additional 'sentry' at the end
            // because fast_expansion_sum_zeroelim() may access
            // an entry past the end (without using it).
            return
                sizeof(expansion) - 2 * sizeof(double) +
                (capa + 1) * sizeof(double);
        }

        expansion(index_t capa) :
            length_(0),
            capacity_(capa) {
        }

#ifdef CPPCHECK
        // cppcheck does not understand that the result
        // of alloca() is passed to the placement syntax
        // of operator new.
    expansion& new_expansion_on_stack(index_t capa);         
#else
#define new_expansion_on_stack(capa)                           \
    (new (alloca(expansion::bytes(capa)))expansion(capa))
#endif

        static expansion* new_expansion_on_heap(index_t capa);

        static void delete_expansion_on_heap(expansion* e);

        // ========================== Initialization from doubles

	expansion& assign(double a) {
	    set_length(1);
	    x_[0] = a;
	    return *this;
	}
	
        static index_t sum_capacity(double a, double b) {
            geo_argused(a);
            geo_argused(b);
            return 2;
        }

        expansion& assign_sum(double a, double b) {
            set_length(2);
            two_sum(a, b, x_[1], x_[0]);
            return *this;
        }

        static index_t diff_capacity(double a, double b) {
            geo_argused(a);
            geo_argused(b);
            return 2;
        }

        expansion& assign_diff(double a, double b) {
            set_length(2);
            two_diff(a, b, x_[1], x_[0]);
            return *this;
        }

        static index_t product_capacity(double a, double b) {
            geo_argused(a);
            geo_argused(b);
            return 2;
        }

        expansion& assign_product(double a, double b) {
            set_length(2);
            two_product(a, b, x_[1], x_[0]);
            return *this;
        }

        static index_t square_capacity(double a) {
            geo_argused(a);
            return 2;
        }

        expansion& assign_square(double a) {
            set_length(2);
            square(a, x_[1], x_[0]);
            return *this;
        }

        // ====== Initialization from expansion and double

        static index_t sum_capacity(const expansion& a, double b) {
            geo_argused(b);
            return a.length() + 1;
        }

        expansion& assign_sum(const expansion& a, double b);

        static index_t diff_capacity(const expansion& a, double b) {
            geo_argused(b);
            return a.length() + 1;
        }

        expansion& assign_diff(const expansion& a, double b);

        static index_t product_capacity(const expansion& a, double b) {
            geo_argused(b);
            // TODO: implement special case where the double argument
            // is a power of two.
            return a.length() * 2;
        }

        expansion& assign_product(const expansion& a, double b);

        // ========================== Initialization from expansions

        static index_t sum_capacity(const expansion& a, const expansion& b) {
            return a.length() + b.length();
        }

        expansion& assign_sum(const expansion& a, const expansion& b);

        static index_t sum_capacity(
            const expansion& a, const expansion& b, const expansion& c
        ) {
            return a.length() + b.length() + c.length();
        }

        expansion& assign_sum(
            const expansion& a, const expansion& b, const expansion& c
        );

        static index_t sum_capacity(
            const expansion& a, const expansion& b,
            const expansion& c, const expansion& d
        ) {
            return a.length() + b.length() + c.length() + d.length();
        }

        expansion& assign_sum(
            const expansion& a, const expansion& b,
            const expansion& c, const expansion& d
        );

        static index_t diff_capacity(const expansion& a, const expansion& b) {
            return a.length() + b.length();
        }

        expansion& assign_diff(const expansion& a, const expansion& b);

        static index_t product_capacity(
            const expansion& a, const expansion& b
        ) {
            return a.length() * b.length() * 2;
        }

        expansion& assign_product(const expansion& a, const expansion& b);

        static index_t product_capacity(
            const expansion& a, const expansion& b, const expansion& c
        ) {
            return a.length() * b.length() * c.length() * 4;
        }

        expansion& assign_product(
            const expansion& a, const expansion& b, const expansion& c
        );

        static index_t square_capacity(const expansion& a) {
            if(a.length() == 2) {
                return 6;
            }                                  // see two_square()
            return a.length() * a.length() * 2;
        }

        expansion& assign_square(const expansion& a);

        // ====== Determinants =============================

        static index_t det2x2_capacity(
            const expansion& a11, const expansion& a12,
            const expansion& a21, const expansion& a22
        ) {
            return
                product_capacity(a11, a22) +
                product_capacity(a21, a12);
        }

        expansion& assign_det2x2(
            const expansion& a11, const expansion& a12,
            const expansion& a21, const expansion& a22
        );

        static index_t det3x3_capacity(
            const expansion& a11, const expansion& a12, const expansion& a13,
            const expansion& a21, const expansion& a22, const expansion& a23,
            const expansion& a31, const expansion& a32, const expansion& a33
        ) {
            // Development w.r.t. first row
            index_t c11_capa = det2x2_capacity(a22, a23, a32, a33);
            index_t c12_capa = det2x2_capacity(a21, a23, a31, a33);
            index_t c13_capa = det2x2_capacity(a21, a22, a31, a32);
            return 2 * (
                a11.length() * c11_capa +
                a12.length() * c12_capa +
                a13.length() * c13_capa
            );
        }

        expansion& assign_det3x3(
            const expansion& a11, const expansion& a12, const expansion& a13,
            const expansion& a21, const expansion& a22, const expansion& a23,
            const expansion& a31, const expansion& a32, const expansion& a33
        );

        static index_t det_111_2x3_capacity(
            const expansion& a21, const expansion& a22, const expansion& a23,
            const expansion& a31, const expansion& a32, const expansion& a33
        ) {
            return
                det2x2_capacity(a22, a23, a32, a33) +
                det2x2_capacity(a23, a21, a33, a31) +
                det2x2_capacity(a21, a22, a31, a32);
        }

        expansion& assign_det_111_2x3(
            const expansion& a21, const expansion& a22, const expansion& a23,
            const expansion& a31, const expansion& a32, const expansion& a33
        );

        // ======= Geometry-specific initializations =======

        static index_t sq_dist_capacity(coord_index_t dim) {
            return index_t(dim) * 6;
        }

        expansion& assign_sq_dist(
            const double* p1, const double* p2, coord_index_t dim
        );

        static index_t dot_at_capacity(coord_index_t dim) {
            return index_t(dim) * 8;
        }

        expansion& assign_dot_at(
            const double* p1, const double* p2, const double* p0,
            coord_index_t dim
        );


        static index_t length2_capacity(
            const expansion& x, const expansion& y, const expansion& z
        ) {
            return square_capacity(x) + square_capacity(y) + square_capacity(z);
        }

        expansion& assign_length2(
            const expansion& x, const expansion& y, const expansion& z
        );
        
        // =============== some general purpose functions =========

        static void initialize();

        expansion& negate() {
            for(index_t i = 0; i < length_; ++i) {
                x_[i] = -x_[i];
            }
            return *this;
        }

        expansion& scale_fast(double s) {
            // TODO: debug assert is_power_of_two(s)
            for(index_t i = 0; i < length_; ++i) {
                x_[i] *= s;
            }
            return *this;
        }

        double estimate() const {
            double result = 0.0;
            for(index_t i = 0; i < length(); ++i) {
                result += x_[i];
            }
            return result;
        }

        Sign sign() const {
            if(length() == 0) {
                return ZERO;
            }
            return geo_sgn(x_[length() - 1]);
        }

        std::ostream& show(std::ostream& os) const {
            for(index_t i = 0; i < length(); ++i) {
                os << i << ':' << x_[i] << ' ';
            }
            return os << std::endl;
        }

    protected:
        static index_t sub_product_capacity(
            index_t a_length, index_t b_length
        ) {
            return a_length * b_length * 2;
        }

        expansion& assign_sub_product(
            const double* a, index_t a_length, const expansion& b
        );

#define expansion_sub_product(a, a_length, b)           \
    new_expansion_on_stack(                       \
        sub_product_capacity(a_length, b.length()) \
    )->assign_sub_product(a, a_length, b)

    private:
        expansion(const expansion& rhs);

        expansion& operator= (const expansion& rhs);

    private:
        index_t length_;
        index_t capacity_;
        double x_[2];  // x_ is in fact of size [capacity_]

        friend class expansion_nt;
    };

    // =============== arithmetic operations ===========================

#define expansion_create(a)	      \
    new_expansion_on_stack(1)->assign(a)

    
#define expansion_sum(a, b)            \
    new_expansion_on_stack(           \
        expansion::sum_capacity(a, b)   \
    )->assign_sum(a, b)

#define expansion_sum3(a, b, c)          \
    new_expansion_on_stack(            \
        expansion::sum_capacity(a, b, c) \
    )->assign_sum(a, b, c)


#define expansion_sum4(a, b, c, d)          \
    new_expansion_on_stack(              \
        expansion::sum_capacity(a, b, c, d) \
    )->assign_sum(a, b, c, d)

#define expansion_diff(a, b)             \
    new_expansion_on_stack(             \
        expansion::diff_capacity(a, b)   \
    )->assign_diff(a, b)

#define expansion_product(a, b)            \
    new_expansion_on_stack(               \
        expansion::product_capacity(a, b)  \
    )->assign_product(a, b)

#define expansion_product3(a, b, c)           \
    new_expansion_on_stack(                 \
        expansion::product_capacity(a, b, c)  \
    )->assign_product(a, b, c)

#define expansion_square(a)             \
    new_expansion_on_stack(             \
        expansion::square_capacity(a)   \
    )->assign_square(a)

    // =============== determinants =====================================

#define expansion_det2x2(a11, a12, a21, a22)          \
    new_expansion_on_stack(                        \
        expansion::det2x2_capacity(a11, a12, a21, a22) \
    )->assign_det2x2(a11, a12, a21, a22)

#define expansion_det3x3(a11, a12, a13, a21, a22, a23, a31, a32, a33)   \
    new_expansion_on_stack(                                             \
        expansion::det3x3_capacity(a11,a12,a13,a21,a22,a23,a31,a32,a33) \
    )->assign_det3x3(a11, a12, a13, a21, a22, a23, a31, a32, a33)

#define expansion_det_111_2x3(a21, a22, a23, a31, a32, a33)           \
    new_expansion_on_stack(                                      \
        expansion::det_111_2x3_capacity(a21, a22, a23, a31, a32, a33) \
    )->assign_det_111_2x3(a21, a22, a23, a31, a32, a33)

    // =============== geometric functions ==============================

#define expansion_sq_dist(a, b, dim)           \
    new_expansion_on_stack(                  \
        expansion::sq_dist_capacity(dim)     \
    )->assign_sq_dist(a, b, dim)

#define expansion_dot_at(a, b, c, dim)           \
    new_expansion_on_stack(                   \
        expansion::dot_at_capacity(dim)       \
    )->assign_dot_at(a, b, c, dim)


#define expansion_length2(x,y,z)              \
    new_expansion_on_stack(                   \
       expansion::length2_capacity(x,y,z)     \
    )->assign_length2(x,y,z)
    
    

    Sign GEOGRAM_API sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,  
        const expansion& a10,const expansion& a11
    );
    
    Sign GEOGRAM_API sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,const expansion& a02,
        const expansion& a10,const expansion& a11,const expansion& a12,
        const expansion& a20,const expansion& a21,const expansion& a22
    );

    Sign GEOGRAM_API sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,
        const expansion& a02,const expansion& a03,
        const expansion& a10,const expansion& a11,
        const expansion& a12,const expansion& a13,
        const expansion& a20,const expansion& a21,
        const expansion& a22,const expansion& a23,
        const expansion& a30,const expansion& a31,
        const expansion& a32,const expansion& a33 
    );
    
    
}

#endif


/******* extracted from expansion_nt.h *******/

#ifndef GEOGRAM_NUMERICS_EXPANSION_NT
#define GEOGRAM_NUMERICS_EXPANSION_NT



namespace GEO {

    class GEOGRAM_API expansion_nt {
    public:
        explicit expansion_nt(double x = 0.0) {
            rep_ = expansion::new_expansion_on_heap(1);
            rep()[0] = x;
            rep().set_length(1);
        }

        expansion_nt(const expansion_nt& rhs) {
            copy(rhs);
        }
        
        expansion_nt& operator= (const expansion_nt& rhs) {
            if(&rhs != this) {
                cleanup();
                copy(rhs);
            }
            return *this;
        }

        ~expansion_nt() {
            cleanup();
        }

        

        expansion_nt& operator+= (const expansion_nt& rhs);

        expansion_nt& operator-= (const expansion_nt& rhs);

        expansion_nt& operator*= (const expansion_nt& rhs);

        expansion_nt& operator+= (double rhs);

        expansion_nt& operator-= (double rhs);

        expansion_nt& operator*= (double rhs);

        

        expansion_nt operator+ (const expansion_nt& rhs) const;

        expansion_nt operator- (const expansion_nt& rhs) const;

        expansion_nt operator* (const expansion_nt& rhs) const;

        expansion_nt operator+ (double rhs) const;

        expansion_nt operator- (double rhs) const;

        expansion_nt operator* (double rhs) const;

        

        expansion_nt operator- () const;

        

        bool operator> (const expansion_nt& rhs) const;

        bool operator>= (const expansion_nt& rhs) const;

        bool operator< (const expansion_nt& rhs) const;

        bool operator<= (const expansion_nt& rhs) const;

        bool operator> (double rhs) const;

        bool operator>= (double rhs) const;

        bool operator< (double rhs) const;

        bool operator<= (double rhs) const;

        

        double estimate() const {
            return rep().estimate();
        }
        
        Sign sign() const {
            return rep().sign();
        }

        index_t length() const {
            return rep().length();
        }

        double component(index_t i) const {
            geo_debug_assert(i < length());
            return rep()[i];
        }
        
        expansion_nt(expansion* rep) :
            rep_(rep) {
        }

        expansion& rep() {
            return *rep_;
        }

        const expansion& rep() const {
            return *rep_;
        }


    protected:

        void copy(const expansion_nt& rhs) {
            rep_ = expansion::new_expansion_on_heap(rhs.rep().capacity());
            rep_->set_length(rhs.rep().length());
            for(index_t i=0; i<rep_->length(); ++i) {
                (*rep_)[i] = rhs.rep()[i];
            }
        }

        void cleanup() {
            expansion::delete_expansion_on_heap(rep_);
            rep_ = nullptr;
        }
        
    private:
        expansion* rep_;
        friend expansion_nt operator- (double a, const expansion_nt& b);

        friend expansion_nt expansion_nt_sq_dist(
            const double* a, const double* b, coord_index_t dim
        );

        friend expansion_nt expansion_nt_dot_at(
            const double* a, const double* b, const double* c,
            coord_index_t dim
        );
    };

    inline expansion_nt operator+ (double a, const expansion_nt& b) {
        return b + a;
    }

    inline expansion_nt operator- (double a, const expansion_nt& b) {
        expansion_nt result = b - a;
        result.rep().negate();
        return result;
    }

    inline expansion_nt operator* (double a, const expansion_nt& b) {
        return b * a;
    }

    inline bool operator== (const expansion_nt& a, const expansion_nt& b) {
        return (a - b).sign() == ZERO;
    }

    inline bool operator== (const expansion_nt& a, double b) {
        return (a - b).sign() == ZERO;
    }

    inline bool operator== (double a, const expansion_nt& b) {
        return (a - b).sign() == ZERO;
    }

    inline bool operator!= (const expansion_nt& a, const expansion_nt& b) {
        return (a - b).sign() != ZERO;
    }

    inline bool operator!= (const expansion_nt& a, double b) {
        return (a - b).sign() != ZERO;
    }

    inline bool operator!= (double a, const expansion_nt& b) {
        return (a - b).sign() != ZERO;
    }

    inline expansion_nt expansion_nt_sq_dist(
        const double* a, const double* b, coord_index_t dim
    ) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::sq_dist_capacity(dim)
        );
        result->assign_sq_dist(a, b, dim);
        return expansion_nt(result);
    }

    inline expansion_nt expansion_nt_dot_at(
        const double* a, const double* b, const double* c, coord_index_t dim
    ) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::dot_at_capacity(dim)
        );
        result->assign_dot_at(a, b, c, dim);
        return expansion_nt(result);
    }

    

    template <> inline Sign geo_sgn(const expansion_nt& x) {
        return x.sign();
    }

    

    inline bool expansion_nt_is_zero(const expansion_nt& x) {
        return (x.sign() == GEO::ZERO);
    }

    inline bool expansion_nt_is_one(const expansion_nt& x) {
        const GEO::expansion& diff = expansion_diff(x.rep(), 1.0);
        return (diff.sign() == GEO::ZERO);
    }


    inline Sign expansion_nt_compare(
        const expansion_nt& x, const expansion_nt& y
    ) {
        const expansion& diff = expansion_diff(x.rep(), y.rep());
        return diff.sign();
    }

    inline expansion_nt expansion_nt_square(const expansion_nt& x) {
        expansion_nt result(
            expansion::new_expansion_on_heap(
                expansion::square_capacity(x.rep()
             ))
        );
        result.rep().assign_square(x.rep());
        return result;
    }


    expansion_nt GEOGRAM_API expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,  
        const expansion_nt& a10,const expansion_nt& a11
    );
    
    expansion_nt GEOGRAM_API expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,const expansion_nt& a02,
        const expansion_nt& a10,const expansion_nt& a11,const expansion_nt& a12,
        const expansion_nt& a20,const expansion_nt& a21,const expansion_nt& a22
    );

    expansion_nt GEOGRAM_API expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,
        const expansion_nt& a02,const expansion_nt& a03,
        const expansion_nt& a10,const expansion_nt& a11,
        const expansion_nt& a12,const expansion_nt& a13,
        const expansion_nt& a20,const expansion_nt& a21,
        const expansion_nt& a22,const expansion_nt& a23,
        const expansion_nt& a30,const expansion_nt& a31,
        const expansion_nt& a32,const expansion_nt& a33 
    );
    
    
}

inline std::ostream& operator<< (
    std::ostream& os, const GEO::expansion_nt& a
) {
    return os << a.estimate();
}

inline std::istream& operator>> ( std::istream& is, GEO::expansion_nt& a) {
    double d;
    is >> d;
    if (is) {
        a = GEO::expansion_nt(d);
    }
    return is;
}



namespace GEO {

    class GEOGRAM_API rational_nt {
      public:

        explicit rational_nt(double x = 0.0) : num_(x), denom_(1.0) {
        }

        explicit rational_nt(const expansion_nt& x) : num_(x), denom_(1.0) {
        }

        explicit rational_nt(const expansion_nt& num, const expansion_nt& denom)
	    : num_(num), denom_(denom) {
        }
	
        rational_nt(const rational_nt& rhs) {
            copy(rhs);
        }
        
        rational_nt& operator= (const rational_nt& rhs) {
            if(&rhs != this) {
                copy(rhs);
            }
            return *this;
        }

        ~rational_nt() {
        }

	const expansion_nt& num() const {
	    return num_;
	}

	const expansion_nt& denom() const {
	    return denom_;
	}

	 expansion_nt& num() {
	    return num_;
	}

	 expansion_nt& denom() {
	    return denom_;
	}
	
        

        rational_nt& operator+= (const rational_nt& rhs) {
	    if(trivially_has_same_denom(rhs)) {
		num_ += rhs.num_;
	    } else {
		num_ = num_ * rhs.denom_ + rhs.num_ * denom_;	    
		denom_ *= rhs.denom_;
	    }
	    return *this;
	}

        rational_nt& operator-= (const rational_nt& rhs) {
	    if(trivially_has_same_denom(rhs)) {
		num_ -= rhs.num_;
	    } else {
		num_ = num_ * rhs.denom_ - rhs.num_ * denom_;	    
		denom_ *= rhs.denom_;
	    }
	    return *this;
	}

        rational_nt& operator*= (const rational_nt& rhs) {
	    num_ *= rhs.num_;
	    denom_ *= rhs.denom_;
	    return *this;
	}

        rational_nt& operator/= (const rational_nt& rhs) {
	    num_ *= rhs.denom_;
	    denom_ *= rhs.num_;
	    return *this;
	}
	
        rational_nt& operator+= (double rhs) {
	    num_ += denom_ * rhs;
	    return *this;
	}

        rational_nt& operator-= (double rhs) {
	    num_ -= denom_ * rhs;
	    return *this;
	}

        rational_nt& operator*= (double rhs) {
	    num_ *= rhs;
	    return *this;
	}

        rational_nt& operator/= (double rhs) {
	    denom_ *= rhs;
	    return *this;
	}
	
        

        rational_nt operator+ (const rational_nt& rhs) const {
	    if(trivially_has_same_denom(rhs)) {
		return rational_nt(
		    num_ + rhs.num_,
		    denom_
		);
	    }
	    return rational_nt(
		num_ * rhs.denom_ + rhs.num_ * denom_,
		denom_ * rhs.denom_
	    );
	}

        rational_nt operator- (const rational_nt& rhs) const {
	    if(trivially_has_same_denom(rhs)) {
		return rational_nt(
		    num_ - rhs.num_,
		    denom_
		);
	    }
	    return rational_nt(
		num_ * rhs.denom_ - rhs.num_ * denom_,
		denom_ * rhs.denom_
	    );
	}

        rational_nt operator* (const rational_nt& rhs) const {
	    return rational_nt(
		num_ * rhs.num_,
		denom_ * rhs.denom_
	    );
	}

        rational_nt operator/ (const rational_nt& rhs) const {
	    return rational_nt(
		num_ * rhs.denom_,
		denom_ * rhs.num_
	    );
	}

	
        rational_nt operator+ (double rhs) const {
	    return rational_nt(
		num_ + rhs * denom_,
		denom_
	    );
	}

        rational_nt operator- (double rhs) const {
	    return rational_nt(
		num_ - rhs * denom_,
		denom_
	    );
	}

        rational_nt operator* (double rhs) const {
	    return rational_nt(
		num_ *rhs,
		denom_
	    );
	}

        rational_nt operator/ (double rhs) const {
	    return rational_nt(
		num_,
		denom_*rhs
	    );
	}
	
        

        rational_nt operator- () const {
	    return rational_nt(
		-num_, 
		denom_
	    );
	}

        

        bool operator> (const rational_nt& rhs) const;

        bool operator>= (const rational_nt& rhs) const;

        bool operator< (const rational_nt& rhs) const;

        bool operator<= (const rational_nt& rhs) const;

        bool operator> (double rhs) const;

        bool operator>= (double rhs) const;

        bool operator< (double rhs) const;

        bool operator<= (double rhs) const;

        

        double estimate() const {
            return num_.estimate() / denom_.estimate();
        }
        
        Sign sign() const {
            return Sign(num_.sign() * denom_.sign());
        }

      protected:
	void copy(const rational_nt& rhs) {
	    num_ = rhs.num_;
	    denom_ = rhs.denom_;
	}

	bool trivially_has_same_denom(const rational_nt& rhs) const {
	    return(
		denom_.rep().length() == 1 &&
		rhs.denom_.rep().length() == 1 &&
		denom_.component(0) == rhs.denom_.component(0)
	    );
	}
	
      private:
	expansion_nt num_;
	expansion_nt denom_;
    };

    

    inline rational_nt operator+ (double a, const rational_nt& b) {
        return b + a;
    }

    inline rational_nt operator- (double a, const rational_nt& b) {
        rational_nt result = b - a;
        result.num().rep().negate();
        return result;
    }

    inline rational_nt operator* (double a, const rational_nt& b) {
        return b * a;
    }

    inline rational_nt operator/ (double a, const rational_nt& b) {
        return rational_nt(
	    a*b.denom(),
	    b.num()
	);
    }
    
    inline bool operator== (const rational_nt& a, const rational_nt& b) {
        return (a - b).sign() == ZERO;
    }

    inline bool operator== (const rational_nt& a, double b) {
        return (a - b).sign() == ZERO;
    }

    inline bool operator== (double a, const rational_nt& b) {
        return (a - b).sign() == ZERO;
    }

    inline bool operator!= (const rational_nt& a, const rational_nt& b) {
        return (a - b).sign() != ZERO;
    }

    inline bool operator!= (const rational_nt& a, double b) {
        return (a - b).sign() != ZERO;
    }

    inline bool operator!= (double a, const rational_nt& b) {
        return (a - b).sign() != ZERO;
    }

    
    
}

#endif
