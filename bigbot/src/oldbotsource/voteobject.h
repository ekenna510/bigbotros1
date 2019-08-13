#ifndef VOTEOBJECT_H
#define VOTEOBJECT_H
#include <string.h>


    class VoteObject
    {

private:
    int MyMax(double votes[],int Index1, int Index2);

public:
        // these are the 7 possible actions
    double SpinLeft=0,HardLeft=0,SoftLeft=0,Straight=0,SoftRight=0,HardRight=0,SpinRight=0;
        // This is the amount of weighting given to this vote object
    double WeightFactor;

    string ToString();
    void Accumulate(VoteObject pVote);
    void Factor();
    int VoteNumber();
    string Vote();
    double MaxValue();

    };

#endif // VOTEOBJECT_H

