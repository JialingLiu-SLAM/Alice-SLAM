//
//  ac_extended.hpp
//  VINS_MapFusion
//
//  Created by 张剑华 on 2023/4/26.
//  Copyright © 2023 zx. All rights reserved.
//

#ifndef ac_extended_hpp
#define ac_extended_hpp

#include <stdio.h>


#pragma once
#include <vector>
#include <list>
#include <stdlib.h>


#define AC_PRECISION 5000


namespace LBFC2
{



typedef struct {
  long low;
  long high;
  long fbits;
  int buffer;
  int bits_to_go;
  long total_bits;
  std::vector<unsigned char> *bitstream;
} ac_encoder;


typedef struct {
  long value;
  long low;
  long high;
  int buffer;
  int bits_to_go;
  int garbage_bits;
  std::list<unsigned char> *bitstream;
} ac_decoder;


typedef struct {
  int nsym;
  int *freq;
  int *cfreq;
  int adapt;
} ac_model;



void output_bit (ac_encoder *ace, int bit);
int input_bit (ac_decoder *acd);
void ac_encoder_init (ac_encoder *, std::vector<unsigned char> &);
void ac_encoder_done (ac_encoder *);
void ac_decoder_init (ac_decoder *, std::list<unsigned char> &);
void ac_decoder_done (ac_decoder *);
void ac_model_init (ac_model *, int, int *, int);
void ac_model_done (ac_model *);
long ac_encoder_bits (ac_encoder *);
void ac_encode_symbol (ac_encoder *, ac_model *, int);
int ac_decode_symbol (ac_decoder *, ac_model *);

double * ac_encode_context (int sym);

double *
ac_encode_context (int sym,int d_last ,double *p_last);

void ac_encode_symbol_updateModel(ac_encoder *, ac_model *, int, int*);
int  ac_decode_symbol_updateModel(ac_decoder *, ac_model *, int*);


struct ACEncodeContext
{
    ACEncodeContext()
    {
        ac_encoder_init (&ace, bitstream);
    }

    ~ACEncodeContext()
    {
        // Intra
        ac_model_done(&acm_bow);
        ac_model_done(&acm_intra_desc);
        ac_model_done(&acm_intra_angle);
        ac_model_done(&acm_intra_octave);

        for( auto &model : v_acm_intra_kpt_x )
            ac_model_done(&model);
        for( auto &model : v_acm_intra_kpt_y )
            ac_model_done(&model);

        // Inter
        ac_model_done(&acm_inter_desc);
        ac_model_done(&acm_inter_angle);
        ac_model_done(&acm_inter_octave);
        ac_model_done(&acm_inter_candidate);
        for( auto &model : v_acm_inter_kpt )
            ac_model_done(&model);


//        ac_model_done(&acm_stereo_candidate);
//        ac_model_done(&acm_stereo_desc);
//
//        for( auto &model : v_acm_stereo_kpt_x )
//            ac_model_done(&model);
//
//        for( auto &model : v_acm_stereo_kpt_y )
//                ac_model_done(&model);



        finish();
    }

    void finish()
    {
        ac_encoder_done(&ace);
    }

    void clear()
    {
        bitstream.clear();
        ac_encoder_init (&ace, bitstream);
    }

    size_t bits()
    {
        return ace.total_bits;
    }

    // Setup encoder for descriptors
    ac_encoder ace;

    // Intra
    ac_model   acm_bow;
    ac_model   acm_bow_ten;
    ac_model   acm_intra_desc;
    ac_model   acm_intra_angle;
    ac_model   acm_intra_octave;
    std::vector<ac_model>   v_acm_intra_kpt_x;
    std::vector<ac_model>   v_acm_intra_kpt_y;

    // Inter
    ac_model   acm_inter_desc;
    ac_model   acm_inter_angle;
    ac_model   acm_inter_octave;
    ac_model   acm_inter_candidate;
    std::vector<ac_model>   v_acm_inter_kpt;

    // Stereo
//    bool bStereo = false;
//    ac_model   acm_stereo_candidate;
//    ac_model   acm_stereo_desc;
//    std::vector<ac_model>   v_acm_stereo_kpt_x;
//    std::vector<ac_model>   v_acm_stereo_kpt_y;


    std::vector<unsigned char> bitstream;
};

struct EncodeContext
{
    EncodeContext(){};
    ~EncodeContext()
    {
        finish();
    }

    size_t bits()
    {
        return (bitstream.size() * 8 + 7-bit_idx);
    }


    void finish()
    {
        // append the remaining bits, if any
        if( bit_idx!=7 ){
            bitstream.push_back(buffer);
        }
    }

    void clear()
    {
        cur_byte = 0;
        bit_idx = 7;
        cur_bit = 0;
        buffer = 0;
        bitstream.clear();
    }

    unsigned char cur_byte = 0;
    int bit_idx = 7;
    int cur_bit = 0;
    unsigned char buffer = 0;
    std::vector<unsigned char> bitstream;//1个字节 8位
};

struct ACDecodeContext
{
    ACDecodeContext(){};
    ~ACDecodeContext()
    {
        // Intra
        ac_model_done(&acm_bow);
        ac_model_done(&acm_intra_desc);
        ac_model_done(&acm_intra_angle);
        ac_model_done(&acm_intra_octave);

        for( auto &model : v_acm_intra_kpt_x )
            ac_model_done(&model);
        for( auto &model : v_acm_intra_kpt_y )
            ac_model_done(&model);

        // Inter
        ac_model_done(&acm_inter_desc);
        ac_model_done(&acm_inter_angle);
        ac_model_done(&acm_inter_octave);
        ac_model_done(&acm_inter_candidate);
        for( auto &model : v_acm_inter_kpt )
            ac_model_done(&model);


        ac_model_done(&acm_stereo_candidate);
        ac_model_done(&acm_stereo_desc);

        for( auto &model : v_acm_stereo_kpt_x )
            ac_model_done(&model);

        for( auto &model : v_acm_stereo_kpt_y )
                ac_model_done(&model);


        ac_decoder_done(&acd);
    }

    void setBitstream( std::list<unsigned char> &_bitstream )
    {
        ac_decoder_init (&acd, _bitstream);
    }


    // If arithmetic coder
    ac_decoder acd;

    // Intra
    ac_model   acm_bow;
    ac_model   acm_intra_desc;
    ac_model   acm_intra_angle;
    ac_model   acm_intra_octave;
    std::vector<ac_model>   v_acm_intra_kpt_x;
    std::vector<ac_model>   v_acm_intra_kpt_y;

    // Inter
    ac_model   acm_inter_desc;
    ac_model   acm_inter_angle;
    ac_model   acm_inter_octave;
    ac_model   acm_inter_candidate;
    std::vector<ac_model>   v_acm_inter_kpt;

    // Stereo
    ac_model   acm_stereo_candidate;
    ac_model   acm_stereo_desc;
    std::vector<ac_model>   v_acm_stereo_kpt_x;
    std::vector<ac_model>   v_acm_stereo_kpt_y;
};


struct DecodeContext
{
    void clear()
    {
        cur_byte = 0;
        byte_idx = 0;
        bit_idx = -1;
        cur_bit = 0;
        bitstream.clear();
    }

    unsigned char cur_byte = 0;
    int byte_idx = 0;
    int bit_idx = -1;
    int cur_bit = 0;

    std::vector<unsigned char> bitstream;
};



} // END NAMESPACE

#endif /* ac_extended_hpp */
