/*#ifndef DECTREEPARAH
#define DECTREEPARAH

class _DecisionTreeParabolic{

    public:
        float inference(float);
        _DecisionTreeParabolic(float*, float*);
        ~_DecisionTreeParabolic();

    private:
        bool leaf;
        float value;  //leaf=false => value for comparison; bool=true => center of gaussian
        _DecisionTreeParabolic *left, *right;

};

class DecisionTreeParabolic{

    public:
        float inference(float, float);
         DecisionTreeParabolic(float*, float*, float*, float*);
         ~DecisionTreeParabolic();
       
    
    private:
        _DecisionTreeParabolic leftSide, rightSide;

};

#endif
*/
#ifndef DECTREEPARAH
#define DECTREEPARAH

#include <tuple>

typedef std::tuple<float, float> CoordAndT;

class _DecisionTreeParabolic{

    public:
        CoordAndT inference(float);
        _DecisionTreeParabolic(CoordAndT*, CoordAndT*);
        ~_DecisionTreeParabolic();

    private:
        bool leaf;
        float value;  //leaf=false => value for comparison; bool=true => center of gaussian
        float t;
        _DecisionTreeParabolic *left, *right;

};

class DecisionTreeParabolic{

    public:
        CoordAndT inference(float, float);
        DecisionTreeParabolic(CoordAndT*, CoordAndT*, CoordAndT*, CoordAndT*);
        ~DecisionTreeParabolic();
       
    
    private:
        _DecisionTreeParabolic leftSide, rightSide;

};

#endif
