#include "MultiPrecision_psm.h"

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



/******* extracted from multi_precision.cpp *******/


// This makes sure the compiler will not optimize y = a*x+b
// with fused multiply-add, this would break the exact
// predicates.
#ifdef GEO_COMPILER_MSVC
#pragma fp_contract(off)
#endif


namespace {

    using namespace GEO;

    
    
    bool expansion_length_stat_ = false;
    std::vector<index_t> expansion_length_histo_;

    class ExpansionStatsDisplay {
    public:
        ~ExpansionStatsDisplay() {
            for(index_t i = 0; i < expansion_length_histo_.size(); ++i) {
                std::cerr << "expansion len " << i
                    << " : " << expansion_length_histo_[i] << std::endl;
            }
        }
    };

    ExpansionStatsDisplay expansion_stats_display_;

    

    class Pools {

    public:
        
        Pools() : pools_(1024,nullptr) {
            chunks_.reserve(1024);
        }

        ~Pools() {
            for(index_t i=0; i<chunks_.size(); ++i) {
                delete[] chunks_[i];
            }
        }

        void* malloc(size_t size) {
            if(size >= pools_.size()) {
                return ::malloc(size);
            } 
            if(pools_[size] == nullptr) {
                new_chunk(size);
            }
            Memory::pointer result = pools_[size];
            pools_[size] = next(pools_[size]);
            return result;
        }

        void free(void* ptr, size_t size) {
            if(size >= pools_.size()) {
                ::free(ptr);
                return;
            }
            set_next(Memory::pointer(ptr), pools_[size]);
            pools_[size] = Memory::pointer(ptr);
        }

        
    protected:
        static const index_t NB_ITEMS_PER_CHUNK = 512;
        
        void new_chunk(size_t item_size) {
            // Allocate chunk
            Memory::pointer chunk =
                new Memory::byte[item_size * NB_ITEMS_PER_CHUNK];
            // Chain items in chunk
            for(index_t i=0; i<NB_ITEMS_PER_CHUNK-1; ++i) {
                Memory::pointer cur_item  = item(chunk, item_size, i);
                Memory::pointer next_item = item(chunk, item_size, i+1);
                set_next(cur_item, next_item);
            }
            // Last item's next is pool's first
            set_next(
                item(chunk, item_size,NB_ITEMS_PER_CHUNK-1),
                pools_[item_size]
            );
            // Set pool's first to first in chunk
            pools_[item_size] = chunk;
            chunks_.push_back(chunk);
        }

    private:

        Memory::pointer next(Memory::pointer item) const {
            return *reinterpret_cast<Memory::pointer*>(item);
        }

        void set_next(
            Memory::pointer item, Memory::pointer next
        ) const {
            *reinterpret_cast<Memory::pointer*>(item) = next;
        }

        Memory::pointer item(
            Memory::pointer chunk, size_t item_size, index_t index
        ) const {
            geo_debug_assert(index < NB_ITEMS_PER_CHUNK);
            return chunk + (item_size * size_t(index));
        }
        
        std::vector<Memory::pointer> pools_;
        
        std::vector<Memory::pointer> chunks_;

    };

    static Pools pools_;
    
    
    
    inline void fast_two_sum(double a, double b, double& x, double& y) {
        x = a + b;
        double bvirt = x - a;
        y = b - bvirt;
    }

#ifdef REMOVE_ME        
    inline void fast_two_diff(double a, double b, double& x, double& y) {
        x = a - b;
        double bvirt = a - x;
        y = bvirt - b;
    }
#endif
    
    inline void two_one_sum(
        double a1, double a0, double b, double& x2, double& x1, double& x0
    ) {
        double _i;
        two_sum(a0, b, _i, x0);
        two_sum(a1, _i, x2, x1);
    }

    inline void two_two_sum(
        double a1, double a0, double b1, double b0,
        double& x3, double& x2, double& x1, double& x0
    ) {
        double _j, _0;
        two_one_sum(a1, a0, b0, _j, _0, x0);
        two_one_sum(_j, _0, b1, x3, x2, x1);
    }

    inline void two_product_presplit(
        double a, double b, double bhi, double blo, double& x, double& y
    ) {
        x = a * b;
        double ahi;
        double alo;
        split(a, ahi, alo);
        double err1 = x - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        y = (alo * blo) - err3;
    }

    inline void two_product_2presplit(
        double a, double ahi, double alo,
        double b, double bhi, double blo,
        double& x, double& y
    ) {
        x = a * b;
        double err1 = x - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        y = (alo * blo) - err3;
    }

    inline void two_square(
        double a1, double a0,
        double* x
    ) {
        double _0, _1, _2;
        double _j, _k, _l;
        square(a0, _j, x[0]);
        _0 = a0 + a0;
        two_product(a1, _0, _k, _1);
        two_one_sum(_k, _1, _j, _l, _2, x[1]);
        square(a1, _j, _1);
        two_two_sum(_j, _1, _l, _2, x[5], x[4], x[3], x[2]);
    }

    void two_two_product(
        const double* a,
        const double* b,
        double* x
    ) {
        double _0, _1, _2;
        double _i, _j, _k, _l, _m, _n;

        // If the target processor supports the FMA (Fused Multiply Add)
        // instruction, then the product of two doubles into a length-2
        // expansion can be implemented as follows. Thanks to Marc Glisse
        // for the information.
        // Note: under gcc, automatic generations of fma() for a*b+c needs
        // to be deactivated, using -ffp-contract=off, else it may break
        // other functions such as fast_expansion_sum_zeroelim().
#ifdef FP_FAST_FMA
        two_product(a[0],b[0],_i,x[0]);
        two_product(a[1],b[0],_j,_0);
        two_sum(_i, _0, _k, _1);
        fast_two_sum(_j, _k, _l, _2);
        two_product(a[0], b[1], _i, _0);
        two_sum(_1, _0, _k, x[1]);
        two_sum(_2, _k, _j, _1);
        two_sum(_l, _j, _m, _2);
        two_product(a[1], b[1], _j, _0);
        two_sum(_i, _0, _n, _0);
        two_sum(_1, _0, _i, x[2]);
        two_sum(_2, _i, _k, _1);
        two_sum(_m, _k, _l, _2);
        two_sum(_j, _n, _k, _0);
        two_sum(_1, _0, _j, x[3]);
        two_sum(_2, _j, _i, _1);
        two_sum(_l, _i, _m, _2);
        two_sum(_1, _k, _i, x[4]);
        two_sum(_2, _i, _k, x[5]);
        two_sum(_m, _k, x[7], x[6]);
#else
        double a0hi, a0lo;
        split(a[0], a0hi, a0lo);
        double bhi, blo;
        split(b[0], bhi, blo);
        two_product_2presplit(
            a[0], a0hi, a0lo, b[0], bhi, blo, _i, x[0]
        );
        double a1hi, a1lo;
        split(a[1], a1hi, a1lo);
        two_product_2presplit(
            a[1], a1hi, a1lo, b[0], bhi, blo, _j, _0
        );
        two_sum(_i, _0, _k, _1);
        fast_two_sum(_j, _k, _l, _2);
        split(b[1], bhi, blo);
        two_product_2presplit(
            a[0], a0hi, a0lo, b[1], bhi, blo, _i, _0
        );
        two_sum(_1, _0, _k, x[1]);
        two_sum(_2, _k, _j, _1);
        two_sum(_l, _j, _m, _2);
        two_product_2presplit(
            a[1], a1hi, a1lo, b[1], bhi, blo, _j, _0
        );
        two_sum(_i, _0, _n, _0);
        two_sum(_1, _0, _i, x[2]);
        two_sum(_2, _i, _k, _1);
        two_sum(_m, _k, _l, _2);
        two_sum(_j, _n, _k, _0);
        two_sum(_1, _0, _j, x[3]);
        two_sum(_2, _j, _i, _1);
        two_sum(_l, _i, _m, _2);
        two_sum(_1, _k, _i, x[4]);
        two_sum(_2, _i, _k, x[5]);
        two_sum(_m, _k, x[7], x[6]);
#endif
    }
}

namespace GEO {

    void grow_expansion_zeroelim(
        const expansion& e, double b, expansion& h
    ) {
        double Q, hh;
        double Qnew;
        index_t eindex, hindex;
        index_t elen = e.length();

        hindex = 0;
        Q = b;
        for(eindex = 0; eindex < elen; eindex++) {
            double enow = e[eindex];
            two_sum(Q, enow, Qnew, hh);
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

    void scale_expansion_zeroelim(
        const expansion& e, double b, expansion& h
    ) {
        double Q, sum;
        double hh;
        double product1;
        double product0;
        index_t eindex, hindex;

        // If the target processor supports the FMA (Fused Multiply Add)
        // instruction, then the product of two doubles into a length-2
        // expansion can be implemented as follows. Thanks to Marc Glisse
        // for the information.
        // Note: under gcc, automatic generations of fma() for a*b+c needs
        // to be deactivated, using -ffp-contract=off, else it may break
        // other functions such as fast_expansion_sum_zeroelim().
#ifndef FP_FAST_FMA
        double bhi, blo;
#endif
        index_t elen = e.length();

        // Sanity check: e and h cannot be the same.
        geo_debug_assert(&e != &h);

#ifdef FP_FAST_FMA
        two_product(e[0], b, Q, hh);
#else
        split(b, bhi, blo);
        two_product_presplit(e[0], b, bhi, blo, Q, hh);
#endif

        hindex = 0;
        if(hh != 0) {
            h[hindex++] = hh;
        }
        for(eindex = 1; eindex < elen; eindex++) {
            double enow = e[eindex];
#ifdef FP_FAST_FMA
            two_product(enow, b,  product1, product0);
#else
            two_product_presplit(enow, b, bhi, blo, product1, product0);
#endif
            two_sum(Q, product0, sum, hh);
            if(hh != 0) {
                h[hindex++] = hh;
            }
            fast_two_sum(product1, sum, Q, hh);
            if(hh != 0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

    void fast_expansion_sum_zeroelim(
        const expansion& e, const expansion& f, expansion& h
    ) {
        double Q;
        double Qnew;
        double hh;
        index_t eindex, findex, hindex;
        double enow, fnow;
        index_t elen = e.length();
        index_t flen = f.length();

        // sanity check: h cannot be e or f
        geo_debug_assert(&h != &e);
        geo_debug_assert(&h != &f);

        enow = e[0];
        fnow = f[0];
        eindex = findex = 0;
        if((fnow > enow) == (fnow > -enow)) {
            Q = enow;
            enow = e[++eindex];
        } else {
            Q = fnow;
            fnow = f[++findex];
        }
        hindex = 0;
        if((eindex < elen) && (findex < flen)) {
            if((fnow > enow) == (fnow > -enow)) {
                fast_two_sum(enow, Q, Qnew, hh);
                enow = e[++eindex];
            } else {
                fast_two_sum(fnow, Q, Qnew, hh);
                fnow = f[++findex];
            }
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
            while((eindex < elen) && (findex < flen)) {
                if((fnow > enow) == (fnow > -enow)) {
                    two_sum(Q, enow, Qnew, hh);
                    enow = e[++eindex];
                } else {
                    two_sum(Q, fnow, Qnew, hh);
                    fnow = f[++findex];
                }
                Q = Qnew;
                if(hh != 0.0) {
                    h[hindex++] = hh;
                }
            }
        }
        while(eindex < elen) {
            two_sum(Q, enow, Qnew, hh);
            enow = e[++eindex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        while(findex < flen) {
            two_sum(Q, fnow, Qnew, hh);
            fnow = f[++findex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }

    void fast_expansion_diff_zeroelim(
        const expansion& e, const expansion& f, expansion& h
    ) {
        double Q;
        double Qnew;
        double hh;
        index_t eindex, findex, hindex;
        double enow, fnow;
        index_t elen = e.length();
        index_t flen = f.length();

        // sanity check: h cannot be e or f
        geo_debug_assert(&h != &e);
        geo_debug_assert(&h != &f);

        enow = e[0];
        fnow = -f[0];
        eindex = findex = 0;
        if((fnow > enow) == (fnow > -enow)) {
            Q = enow;
            enow = e[++eindex];
        } else {
            Q = fnow;
            fnow = -f[++findex];
        }
        hindex = 0;
        if((eindex < elen) && (findex < flen)) {
            if((fnow > enow) == (fnow > -enow)) {
                fast_two_sum(enow, Q, Qnew, hh);
                enow = e[++eindex];
            } else {
                fast_two_sum(fnow, Q, Qnew, hh);
                fnow = -f[++findex];
            }
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
            while((eindex < elen) && (findex < flen)) {
                if((fnow > enow) == (fnow > -enow)) {
                    two_sum(Q, enow, Qnew, hh);
                    enow = e[++eindex];
                } else {
                    two_sum(Q, fnow, Qnew, hh);
                    fnow = -f[++findex];
                }
                Q = Qnew;
                if(hh != 0.0) {
                    h[hindex++] = hh;
                }
            }
        }
        while(eindex < elen) {
            two_sum(Q, enow, Qnew, hh);
            enow = e[++eindex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        while(findex < flen) {
            two_sum(Q, fnow, Qnew, hh);
            fnow = -f[++findex];
            Q = Qnew;
            if(hh != 0.0) {
                h[hindex++] = hh;
            }
        }
        if((Q != 0.0) || (hindex == 0)) {
            h[hindex++] = Q;
        }
        h.set_length(hindex);
    }
}



namespace GEO {

    double expansion_splitter_;
    double expansion_epsilon_;

    void expansion::initialize() {
        // Taken from Jonathan Shewchuk's exactinit.
        double half;
        double check, lastcheck;
        int every_other;

        every_other = 1;
        half = 0.5;
        expansion_epsilon_ = 1.0;
        expansion_splitter_ = 1.0;
        check = 1.0;
        // Repeatedly divide `epsilon' by two until it is too small to add to
        // one without causing roundoff.  (Also check if the sum is equal to
        // the previous sum, for machines that round up instead of using exact
        // rounding.  Not that this library will work on such machines anyway.
        do {
            lastcheck = check;
            expansion_epsilon_ *= half;
            if(every_other) {
                expansion_splitter_ *= 2.0;
            }
            every_other = !every_other;
            check = 1.0 + expansion_epsilon_;
        } while((check != 1.0) && (check != lastcheck));
        expansion_splitter_ += 1.0;
    }

    static Process::spinlock expansions_lock = GEOGRAM_SPINLOCK_INIT;
    
    expansion* expansion::new_expansion_on_heap(index_t capa) {
	Process::acquire_spinlock(expansions_lock);
        if(expansion_length_stat_) {
            if(capa >= expansion_length_histo_.size()) {
                expansion_length_histo_.resize(capa + 1);
            }
            expansion_length_histo_[capa]++;
        }
        Memory::pointer addr = Memory::pointer(
            pools_.malloc(expansion::bytes(capa))
        );
	Process::release_spinlock(expansions_lock);
        expansion* result = new(addr)expansion(capa);
        return result;
    }

    void expansion::delete_expansion_on_heap(expansion* e) {
	Process::acquire_spinlock(expansions_lock);	
        pools_.free(e, expansion::bytes(e->capacity()));
	Process::release_spinlock(expansions_lock);	
    }

    // ====== Initialization from expansion and double ===============

    expansion& expansion::assign_sum(const expansion& a, double b) {
        geo_debug_assert(capacity() >= sum_capacity(a, b));
        grow_expansion_zeroelim(a, b, *this);
        return *this;
    }

    expansion& expansion::assign_diff(const expansion& a, double b) {
        geo_debug_assert(capacity() >= diff_capacity(a, b));
        grow_expansion_zeroelim(a, -b, *this);
        return *this;
    }

    expansion& expansion::assign_product(const expansion& a, double b) {
        // TODO: implement special case where the double argument
        // is a power of two.
        geo_debug_assert(capacity() >= product_capacity(a, b));
        scale_expansion_zeroelim(a, b, *this);
        return *this;
    }

    // =============  expansion sum and difference =========================

    expansion& expansion::assign_sum(
        const expansion& a, const expansion& b
    ) {
        geo_debug_assert(capacity() >= sum_capacity(a, b));
        fast_expansion_sum_zeroelim(a, b, *this);
        return *this;
    }

    expansion& expansion::assign_sum(
        const expansion& a, const expansion& b, const expansion& c
    ) {
        geo_debug_assert(capacity() >= sum_capacity(a, b, c));
        expansion& ab = expansion_sum(a, b);
        this->assign_sum(ab, c);
        return *this;
    }

    expansion& expansion::assign_sum(
        const expansion& a, const expansion& b,
        const expansion& c, const expansion& d
    ) {
        geo_debug_assert(capacity() >= sum_capacity(a, b, c));
        expansion& ab = expansion_sum(a, b);
        expansion& cd = expansion_sum(c, d);
        this->assign_sum(ab, cd);
        return *this;
    }

    expansion& expansion::assign_diff(const expansion& a, const expansion& b) {
        geo_debug_assert(capacity() >= diff_capacity(a, b));
        fast_expansion_diff_zeroelim(a, b, *this);
        return *this;
    }

    // =============  expansion product ==================================

    // Recursive helper function for product implementation
    expansion& expansion::assign_sub_product(
        const double* a, index_t a_length, const expansion& b
    ) {
        geo_debug_assert(
            capacity() >= sub_product_capacity(a_length, b.length())
        );
        if(a_length == 1) {
            scale_expansion_zeroelim(b, a[0], *this);
        } else {
            // "Distillation" (see Shewchuk's paper) is computed recursively,
            // by splitting the list of expansions to sum into two halves.
            const double* a1 = a;
            index_t a1_length = a_length / 2;
            const double* a2 = a1 + a1_length;
            index_t a2_length = a_length - a1_length;
            expansion& a1b = expansion_sub_product(a1, a1_length, b);
            expansion& a2b = expansion_sub_product(a2, a2_length, b);
            this->assign_sum(a1b, a2b);
        }
        return *this;
    }

    expansion& expansion::assign_product(
        const expansion& a, const expansion& b
    ) {
        geo_debug_assert(capacity() >= product_capacity(a, b));
        if(a.length() == 0 || b.length() == 0) {
            x_[0] = 0.0;
            set_length(0);
        } else if(a.length() == 1 && b.length() == 1) {
            two_product(a[0], b[0], x_[1], x_[0]);
            set_length(2);
        } else if(a.length() == 1) {
            scale_expansion_zeroelim(b, a[0], *this);
        } else if(b.length() == 1) {
            scale_expansion_zeroelim(a, b[0], *this);
        } else if(a.length() == 2 && b.length() == 2) {
            two_two_product(a.data(), b.data(), x_);
            set_length(8);
        } else {
            // Recursive distillation: the shortest expansion
            // is split into two parts.
            if(a.length() < b.length()) {
                const double* a1 = a.data();
                index_t a1_length = a.length() / 2;
                const double* a2 = a1 + a1_length;
                index_t a2_length = a.length() - a1_length;
                expansion& a1b = expansion_sub_product(a1, a1_length, b);
                expansion& a2b = expansion_sub_product(a2, a2_length, b);
                this->assign_sum(a1b, a2b);
            } else {
                const double* b1 = b.data();
                index_t b1_length = b.length() / 2;
                const double* b2 = b1 + b1_length;
                index_t b2_length = b.length() - b1_length;
                expansion& ab1 = expansion_sub_product(b1, b1_length, a);
                expansion& ab2 = expansion_sub_product(b2, b2_length, a);
                this->assign_sum(ab1, ab2);
            }
        }
        return *this;
    }

    expansion& expansion::assign_product(
        const expansion& a, const expansion& b, const expansion& c
    ) {
        const expansion& bc = expansion_product(b, c);
        this->assign_product(a, bc);
        return *this;
    }

    expansion& expansion::assign_square(const expansion& a) {
        geo_debug_assert(capacity() >= square_capacity(a));
        if(a.length() == 1) {
            square(a[0], x_[1], x_[0]);
            set_length(2);
        } else if(a.length() == 2) {
            two_square(a[1], a[0], x_);
            set_length(6);
        } else {
            this->assign_product(a, a);
        }
        return *this;
    }

    // =============  determinants ==========================================

    expansion& expansion::assign_det2x2(
        const expansion& a11, const expansion& a12,
        const expansion& a21, const expansion& a22
    ) {
        const expansion& a11a22 = expansion_product(a11, a22);
        const expansion& a12a21 = expansion_product(a12, a21);
        return this->assign_diff(a11a22, a12a21);
    }

    expansion& expansion::assign_det3x3(
        const expansion& a11, const expansion& a12, const expansion& a13,
        const expansion& a21, const expansion& a22, const expansion& a23,
        const expansion& a31, const expansion& a32, const expansion& a33
    ) {
        // Development w.r.t. first row
        const expansion& c11 = expansion_det2x2(a22, a23, a32, a33);
        const expansion& c12 = expansion_det2x2(a23, a21, a33, a31);
        const expansion& c13 = expansion_det2x2(a21, a22, a31, a32);
        const expansion& a11c11 = expansion_product(a11, c11);
        const expansion& a12c12 = expansion_product(a12, c12);
        const expansion& a13c13 = expansion_product(a13, c13);
        return this->assign_sum(a11c11, a12c12, a13c13);
    }

    expansion& expansion::assign_det_111_2x3(
        const expansion& a21, const expansion& a22, const expansion& a23,
        const expansion& a31, const expansion& a32, const expansion& a33
    ) {
        const expansion& c11 = expansion_det2x2(a22, a23, a32, a33);
        const expansion& c12 = expansion_det2x2(a23, a21, a33, a31);
        const expansion& c13 = expansion_det2x2(a21, a22, a31, a32);
        return this->assign_sum(c11, c12, c13);
    }

    // =============  geometric operations ==================================

    expansion& expansion::assign_sq_dist(
        const double* p1, const double* p2, coord_index_t dim
    ) {
        geo_debug_assert(capacity() >= sq_dist_capacity(dim));
	geo_debug_assert(dim > 0);
        if(dim == 1) {
            double d0, d1;
            two_diff(p1[0], p2[0], d1, d0);
            two_square(d1, d0, x_);
            set_length(6);
        } else {
            // "Distillation" (see Shewchuk's paper) is computed recursively,
            // by splitting the list of expansions to sum into two halves.
            coord_index_t dim1 = dim / 2;
            coord_index_t dim2 = coord_index_t(dim - dim1);
            const double* p1_2 = p1 + dim1;
            const double* p2_2 = p2 + dim1;
            expansion& d1 = expansion_sq_dist(p1, p2, dim1);
            expansion& d2 = expansion_sq_dist(p1_2, p2_2, dim2);
            this->assign_sum(d1, d2);
        }
        return *this;
    }

    expansion& expansion::assign_dot_at(
        const double* p1, const double* p2, const double* p0,
        coord_index_t dim
    ) {
        geo_debug_assert(capacity() >= dot_at_capacity(dim));
        if(dim == 1) {

            double v[2];
            two_diff(p1[0], p0[0], v[1], v[0]);
            double w[2];
            two_diff(p2[0], p0[0], w[1], w[0]);
            two_two_product(v, w, x_);
            set_length(8);
        } else {
            // "Distillation" (see Shewchuk's paper) is computed recursively,
            // by splitting the list of expansions to sum into two halves.
            coord_index_t dim1 = dim / 2;
            coord_index_t dim2 = coord_index_t(dim - dim1);
            const double* p1_2 = p1 + dim1;
            const double* p2_2 = p2 + dim1;
            const double* p0_2 = p0 + dim1;
            expansion& d1 = expansion_dot_at(p1, p2, p0, dim1);
            expansion& d2 = expansion_dot_at(p1_2, p2_2, p0_2, dim2);
            this->assign_sum(d1, d2);
        } 
        return *this;
    }

    expansion& expansion::assign_length2(
        const expansion& x, const expansion& y, const expansion& z
    ) {
        const expansion& x2 = expansion_square(x);
        const expansion& y2 = expansion_square(y);
        const expansion& z2 = expansion_square(z);
        this->assign_sum(x2,y2,z2);
        return *this;
    }
    
    

    bool expansion::is_same_as(const expansion& rhs) const {
        if(length() != rhs.length()) {
            return false;
        }
        for(index_t i=0; i<length(); ++i) {
            if(x_[i] != rhs.x_[i]) {
                return false;
            }
        }
        return true;
    }

    bool expansion::is_same_as(double rhs) const {
        if(length() != 1) {
            return false;
        }
        return (x_[0] == rhs);
    }

    Sign expansion::compare(const expansion& rhs) const {
        if(is_same_as(rhs)) {
            return ZERO;
        }
        const expansion& d = expansion_diff(*this, rhs);
        return d.sign();
    }
    
    Sign expansion::compare(double rhs) const {
        if(rhs == 0.0) {
            return sign();
        }
        if(is_same_as(rhs)) {
            return ZERO;
        }
        const expansion& d = expansion_diff(*this, rhs);
        return d.sign();
    }
    
    
    
    Sign sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,  
        const expansion& a10,const expansion& a11
    ) {
        const expansion& result = expansion_det2x2(a00, a01, a10, a11);
        return result.sign();
    }

    Sign sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,const expansion& a02,
        const expansion& a10,const expansion& a11,const expansion& a12,
        const expansion& a20,const expansion& a21,const expansion& a22
    ) {
        // First compute the det2x2
        const expansion& m01 =
            expansion_det2x2(a00, a10, a01, a11); 
        const expansion& m02 =
            expansion_det2x2(a00, a20, a01, a21);
        const expansion& m12 =
            expansion_det2x2(a10, a20, a11, a21);

        // Now compute the minors of rank 3
        const expansion& z1 = expansion_product(m01,a22);
        const expansion& z2 = expansion_product(m02,a12).negate();
        const expansion& z3 = expansion_product(m12,a02);

        const expansion& result = expansion_sum3(z1,z2,z3);
        return result.sign();
    }
    
    Sign sign_of_expansion_determinant(
        const expansion& a00,const expansion& a01,
        const expansion& a02,const expansion& a03,
        const expansion& a10,const expansion& a11,
        const expansion& a12,const expansion& a13,
        const expansion& a20,const expansion& a21,
        const expansion& a22,const expansion& a23,
        const expansion& a30,const expansion& a31,
        const expansion& a32,const expansion& a33 
    ) {

        // First compute the det2x2        
        const expansion& m01 =
            expansion_det2x2(a10,a00,a11,a01);
        const expansion& m02 =
            expansion_det2x2(a20,a00,a21,a01);
        const expansion& m03 =
            expansion_det2x2(a30,a00,a31,a01);
        const expansion& m12 =
            expansion_det2x2(a20,a10,a21,a11);
        const expansion& m13 =
            expansion_det2x2(a30,a10,a31,a11);
        const expansion& m23 =
            expansion_det2x2(a30,a20,a31,a21);     
        
        // Now compute the minors of rank 3
        const expansion& m012_1 = expansion_product(m12,a02);
        expansion& m012_2 = expansion_product(m02,a12); m012_2.negate();
        const expansion& m012_3 = expansion_product(m01,a22);
        const expansion& m012 = expansion_sum3(m012_1, m012_2, m012_3);

        const expansion& m013_1 = expansion_product(m13,a02);
        expansion& m013_2 = expansion_product(m03,a12); m013_2.negate();
        
        const expansion& m013_3 = expansion_product(m01,a32);
        const expansion& m013 = expansion_sum3(m013_1, m013_2, m013_3);
        
        const expansion& m023_1 = expansion_product(m23,a02);
        expansion& m023_2 = expansion_product(m03,a22); m023_2.negate();
        const expansion& m023_3 = expansion_product(m02,a32);
        const expansion& m023 = expansion_sum3(m023_1, m023_2, m023_3);

        const expansion& m123_1 = expansion_product(m23,a12);
        expansion& m123_2 = expansion_product(m13,a22); m123_2.negate();
        const expansion& m123_3 = expansion_product(m12,a32);
        const expansion& m123 = expansion_sum3(m123_1, m123_2, m123_3);
        
        // Now compute the minors of rank 4
        const expansion& m0123_1 = expansion_product(m123,a03);
        const expansion& m0123_2 = expansion_product(m023,a13);
        const expansion& m0123_3 = expansion_product(m013,a23);
        const expansion& m0123_4 = expansion_product(m012,a33);

        const expansion& z1 = expansion_sum(m0123_1, m0123_3);
        const expansion& z2 = expansion_sum(m0123_2, m0123_4);

        const expansion& result = expansion_diff(z1,z2);
        return result.sign();
    }
    
    

    void expansion::optimize() {
        grow_expansion_zeroelim(*this, 0.0, *this);
    }

    
    
}


/******* extracted from expansion_nt.cpp *******/


namespace GEO {

    expansion_nt& expansion_nt::operator+= (const expansion_nt& rhs) {
        index_t e_capa = expansion::sum_capacity(rep(), rhs.rep());
        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_sum(rep(), rhs.rep());
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator+= (double rhs) {
        index_t e_capa = expansion::sum_capacity(rep(), rhs);

        // TODO: optimized in-place version to be used
        //   if(!shared() && e_capa < rep().capacity())

        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_sum(rep(), rhs);
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator-= (const expansion_nt& rhs) {
        index_t e_capa = expansion::diff_capacity(rep(), rhs.rep());
        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_diff(rep(), rhs.rep());
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator-= (double rhs) {
        index_t e_capa = expansion::diff_capacity(rep(), rhs);

        // TODO: optimized in-place version to be used
        //   if(!shared() && e_capa < rep().capacity())

        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_diff(rep(), rhs);
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator*= (const expansion_nt& rhs) {
        index_t e_capa = expansion::product_capacity(rep(), rhs.rep());
        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_product(rep(), rhs.rep());
        cleanup();
        rep_ = e;
        return *this;
    }

    expansion_nt& expansion_nt::operator*= (double rhs) {
        index_t e_capa = expansion::product_capacity(rep(), rhs);

        // TODO: optimized in-place version to be used
        //   if(!shared() && e_capa < rep().capacity())

        expansion* e = expansion::new_expansion_on_heap(e_capa);
        e->assign_product(rep(), rhs);
        cleanup();
        rep_ = e;
        return *this;
    }

    

    expansion_nt expansion_nt::operator+ (const expansion_nt& rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::sum_capacity(rep(), rhs.rep())
        );
        e->assign_sum(rep(), rhs.rep());
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator- (const expansion_nt& rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::diff_capacity(rep(), rhs.rep())
        );
        e->assign_diff(rep(), rhs.rep());
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator* (const expansion_nt& rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::product_capacity(rep(), rhs.rep())
        );
        e->assign_product(rep(), rhs.rep());
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator+ (double rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::sum_capacity(rep(), rhs)
        );
        e->assign_sum(rep(), rhs);
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator- (double rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::diff_capacity(rep(), rhs)
        );
        e->assign_diff(rep(), rhs);
        return expansion_nt(e);
    }

    expansion_nt expansion_nt::operator* (double rhs) const {
        expansion* e = expansion::new_expansion_on_heap(
            expansion::product_capacity(rep(), rhs)
        );
        e->assign_product(rep(), rhs);
        return expansion_nt(e);
    }

    

    expansion_nt expansion_nt::operator- () const {
        expansion_nt result(*this);
        result.rep().negate();
        return result;
    }

    

    expansion_nt expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,  
        const expansion_nt& a10,const expansion_nt& a11
    ) {
        expansion* result = expansion::new_expansion_on_heap(
            expansion::det2x2_capacity(a00.rep(),a01.rep(),a10.rep(),a11.rep())
        );
        result->assign_det2x2(a00.rep(),a01.rep(),a10.rep(),a11.rep());
        return expansion_nt(result);
    }


    expansion_nt expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,const expansion_nt& a02,
        const expansion_nt& a10,const expansion_nt& a11,const expansion_nt& a12,
        const expansion_nt& a20,const expansion_nt& a21,const expansion_nt& a22
    ) {
        // First compute the det2x2
        const expansion& m01 =
            expansion_det2x2(a00.rep(), a10.rep(), a01.rep(), a11.rep()); 
        const expansion& m02 =
            expansion_det2x2(a00.rep(), a20.rep(), a01.rep(), a21.rep());
        const expansion& m12 =
            expansion_det2x2(a10.rep(), a20.rep(), a11.rep(), a21.rep());

        // Now compute the minors of rank 3
        const expansion& z1 = expansion_product(m01,a22.rep());
        const expansion& z2 = expansion_product(m02,a12.rep()).negate();
        const expansion& z3 = expansion_product(m12,a02.rep());

        return expansion_nt(expansion_nt::SUM, z1, z2, z3);
    }
    
    expansion_nt expansion_nt_determinant(
        const expansion_nt& a00,const expansion_nt& a01,
        const expansion_nt& a02,const expansion_nt& a03,
        const expansion_nt& a10,const expansion_nt& a11,
        const expansion_nt& a12,const expansion_nt& a13,
        const expansion_nt& a20,const expansion_nt& a21,
        const expansion_nt& a22,const expansion_nt& a23,
        const expansion_nt& a30,const expansion_nt& a31,
        const expansion_nt& a32,const expansion_nt& a33 
    ) {

        // First compute the det2x2        
        const expansion& m01 =
            expansion_det2x2(a10.rep(),a00.rep(),a11.rep(),a01.rep());
        const expansion& m02 =
            expansion_det2x2(a20.rep(),a00.rep(),a21.rep(),a01.rep());
        const expansion& m03 =
            expansion_det2x2(a30.rep(),a00.rep(),a31.rep(),a01.rep());
        const expansion& m12 =
            expansion_det2x2(a20.rep(),a10.rep(),a21.rep(),a11.rep());
        const expansion& m13 =
            expansion_det2x2(a30.rep(),a10.rep(),a31.rep(),a11.rep());
        const expansion& m23 =
            expansion_det2x2(a30.rep(),a20.rep(),a31.rep(),a21.rep());     
        
        // Now compute the minors of rank 3
        const expansion& m012_1 = expansion_product(m12,a02.rep());
        expansion& m012_2 = expansion_product(m02,a12.rep()); m012_2.negate();
        const expansion& m012_3 = expansion_product(m01,a22.rep());
        const expansion& m012 = expansion_sum3(m012_1, m012_2, m012_3);

        const expansion& m013_1 = expansion_product(m13,a02.rep());
        expansion& m013_2 = expansion_product(m03,a12.rep()); m013_2.negate();
        
        const expansion& m013_3 = expansion_product(m01,a32.rep());
        const expansion& m013 = expansion_sum3(m013_1, m013_2, m013_3);
        
        const expansion& m023_1 = expansion_product(m23,a02.rep());
        expansion& m023_2 = expansion_product(m03,a22.rep()); m023_2.negate();
        const expansion& m023_3 = expansion_product(m02,a32.rep());
        const expansion& m023 = expansion_sum3(m023_1, m023_2, m023_3);

        const expansion& m123_1 = expansion_product(m23,a12.rep());
        expansion& m123_2 = expansion_product(m13,a22.rep()); m123_2.negate();
        const expansion& m123_3 = expansion_product(m12,a32.rep());
        const expansion& m123 = expansion_sum3(m123_1, m123_2, m123_3);
        
        // Now compute the minors of rank 4
        const expansion& m0123_1 = expansion_product(m123,a03.rep());
        const expansion& m0123_2 = expansion_product(m023,a13.rep());
        const expansion& m0123_3 = expansion_product(m013,a23.rep());
        const expansion& m0123_4 = expansion_product(m012,a33.rep());

        const expansion& z1 = expansion_sum(m0123_1, m0123_3);
        const expansion& z2 = expansion_sum(m0123_2, m0123_4);

        return expansion_nt(expansion_nt::DIFF,z1,z2);
    }
    
    
    
    Sign rational_nt::compare(const rational_nt& rhs) const {
	if(has_same_denom(rhs)) {
	    const expansion& diff_num = expansion_diff(
		num_.rep(), rhs.num_.rep()
	    );
	    return Sign(diff_num.sign() * denom_.sign());
	}
	const expansion& num_a = expansion_product(
	    num_.rep(), rhs.denom_.rep()
	);
	const expansion& num_b = expansion_product(
	    rhs.num_.rep(), denom_.rep()
	);
	const expansion& diff_num = expansion_diff(num_a, num_b);
	return Sign(
	    diff_num.sign() * denom_.sign() * rhs.denom_.sign()
	);
    }

    Sign rational_nt::compare(double rhs) const {
	const expansion& num_b = expansion_product(
	    denom_.rep(), rhs
	);
	const expansion& diff_num = expansion_diff(num_.rep(), num_b);
	return Sign(diff_num.sign() * denom_.sign());
    }
    
    
}
