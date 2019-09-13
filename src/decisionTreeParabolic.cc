#include "decisionTreeParabolic.h"

#include <iostream>

DecisionTreeParabolic::DecisionTreeParabolic(CoordAndT* beginLeft, CoordAndT* endLeft, CoordAndT* beginRight, CoordAndT* endRight) : leftSide(beginLeft, endLeft), rightSide(beginRight, endRight){}

CoordAndT DecisionTreeParabolic::inference(float y, float zORx){

    return y < 0 ? this-> leftSide.inference(zORx) : this-> rightSide.inference(zORx);

}

CoordAndT _DecisionTreeParabolic::inference(float input){

    if( this -> leaf == true )
        return CoordAndT(this -> value, this -> t);

    if(input < this -> value)
        return this -> left -> inference(input);
    
    return this -> right -> inference(input);

}

_DecisionTreeParabolic::_DecisionTreeParabolic(CoordAndT* begin, CoordAndT* end){

    if(begin == end){
   
        this -> leaf = true;
        this -> value = std::get<0>(*begin);
        this -> t = std::get<1>(*begin);

        return;
    }

    //float *half = (float*)(begin + (end-begin)/2);
    CoordAndT *half = (CoordAndT*)(begin + (end-begin)/2);
   
    this -> leaf = false;
    this -> value = (std::get<0>(*half)+std::get<0>(*(half+1)))/2;
    this -> left = new _DecisionTreeParabolic(begin, half);
    this -> right = new _DecisionTreeParabolic(half+1, end);
}

_DecisionTreeParabolic::~_DecisionTreeParabolic(){

    free(this -> left);
    free(this -> right);

}

DecisionTreeParabolic::~DecisionTreeParabolic(){

}
