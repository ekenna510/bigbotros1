include "VoteObject.h"

        // these are the 7 possible actions

        String VoteObject::ToString()
        {
            return "Vote SL " + SpinLeft.ToString("0.00") +
                " HL " + HardLeft.ToString("0.00") +
                " L " + SoftLeft.ToString("0.00") +
                " St " + Straight.ToString("0.00") +
                " R " + SoftRight.ToString("0.00") +
                " HR " + HardRight.ToString("0.00") +
                " SR " + SpinRight.ToString("0.00");

        }
        // Add the weighted votes from another vote object to this one
        // you need to have run factor first.
        void VoteObject::Accumulate(VoteObject pVote)
        {
            
            SpinLeft += (pVote.SpinLeft * pVote.WeightFactor);
            SpinRight += (pVote.SpinRight * pVote.WeightFactor);
            HardLeft += (pVote.HardLeft * pVote.WeightFactor);
            HardRight += (pVote.HardRight * pVote.WeightFactor);
            SoftLeft += (pVote.SoftLeft * pVote.WeightFactor);
            SoftRight += (pVote.SoftRight * pVote.WeightFactor);
            Straight += (pVote.Straight * pVote.WeightFactor);

        }
        // This converts the votes in this vote object to a weighted vote
        // so other votes can be added to it.
        void VoteObject::Factor()
        {
            SpinLeft  *= WeightFactor;
            SpinRight *= WeightFactor;
            HardLeft *= WeightFactor;
            HardRight *= WeightFactor;
            SoftLeft *= WeightFactor;
            SoftRight *= WeightFactor;
            Straight  *= WeightFactor;
        }
        int VoteObject::VoteNumber()
        {

            double[] Votes = new double[7];
            Votes[0] = SpinLeft;
            Votes[1] = HardLeft;
            Votes[2] = SoftLeft;
            Votes[3] = Straight;
            Votes[4] = SoftRight;
            Votes[5] = HardRight;
            Votes[6] = SpinRight;
            int Max = MyMax(Votes, 0, MyMax(Votes, 1, MyMax(Votes, 2, MyMax(Votes, 3, MyMax(Votes, 4, MyMax(Votes, 5, 6))))));
            return Max;
        }

        string VoteObject::Vote()
        {

            double[] Votes = new double[7];
            Votes[0] = SpinLeft;
            Votes[1] = HardLeft;
            Votes[2] = SoftLeft;
            Votes[3] = Straight;
            Votes[4] = SoftRight;
            Votes[5] = HardRight;
            Votes[6] = SpinRight;
            int Max = MyMax(Votes, 0, MyMax(Votes, 1, MyMax(Votes, 2, MyMax(Votes, 3, MyMax(Votes, 4, MyMax(Votes, 5, 6))))));
            if (Max == 0)
            {
                return "SL";
            }
            else
                if (Max == 1)
                {
                    return "HL";
                }
                else
                    if (Max == 2)
                    {
                        return "L";
                    }
                    else
                        if (Max == 3)
                        {
                            return "ST";
                        }
                        else
                            if (Max == 4)
                            {
                                return "R";
                            }
                            else
                                if (Max == 5)
                                {
                                    return "HR";
                                }
                                else
                                    {
                                        return "SR";
                                    }


        }
        int VoteObject::MyMax(double[] votes,int Index1, int Index2)
        {
            if (votes[Index1] < votes[Index2])
            {
                return Index2;
            }
            else
            {
                return Index1;
            }

        }
        double VoteObject::MaxValue()
        {
            double[] Votes = new double[7];
            Votes[0] = SpinLeft;
            Votes[1] = HardLeft;
            Votes[2] = SoftLeft;
            Votes[3] = Straight;
            Votes[4] = SoftRight;
            Votes[5] = HardRight;
            Votes[6] = SpinRight;
            int Max = MyMax(Votes, 0, MyMax(Votes, 1, MyMax(Votes, 2, MyMax(Votes, 3, MyMax(Votes, 4, MyMax(Votes, 5, 6))))));
            return Votes[Max];
        }
    }
}
