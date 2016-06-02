#ifndef PEMM_RUBIK_H
#define PEMM_RUBIK_H
#include "PEMM.h"

class PEMMRubik : public PEMM<RubiksState, RubiksAction> {
public:
	PEMMRubik(RubiksState &start, RubiksState &goal, const char *p1, const char *p2, Heuristic<RubiksState> f, Heuristic<RubiksState> b, 
		RubiksCube* se, double lambda, int aaf)
		: PEMM<RubiksState, RubiksAction>(start, goal, p1, p2, f, b, se,lambda,aaf)
	{
	}

protected:
	virtual void GetState(RubiksState &s, int bucket, uint64_t data);
	virtual int GetBucket(const RubiksState &s);
	virtual void GetBucketAndData(const RubiksState &s, int &bucket, uint64_t &data);
};

#endif