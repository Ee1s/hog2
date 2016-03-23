#ifndef PEMM_PANCAKE_H
#define PEMM_PANCAKE_H
#include "PEMM.h"
#include "PancakePuzzle.h"
#include "PermutationPDB.h"

class PEMMPancake : public PEMM<PancakePuzzleState, PancakePuzzleAction> {
public:
	PEMMPancake(PancakePuzzleState &start, PancakePuzzleState &goal, const char *p1, const char *p2, Heuristic<PancakePuzzleState>& f, Heuristic<PancakePuzzleState>& b, PancakePuzzle* se)
		: PEMM<PancakePuzzleState, PancakePuzzleAction>(start, goal, p1, p2, f, b, se)
	{
	}

protected:
	virtual void GetState(PancakePuzzleState &s, int bucket, uint64_t data);
	virtual int GetBucket(const PancakePuzzleState &s);
	virtual void GetBucketAndData(const PancakePuzzleState &s, int &bucket, uint64_t &data);
};


class PancakePDB : public PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> {
public:
	PancakePDB(PancakePuzzle* e,const PancakePuzzleState& s, std::vector<int> distincts):
		PermutationPDB(e, s, distincts)
	{
		SetGoal(s);
	}

	static uint64_t GetStateHash(const PancakePuzzleState& s);
	static void GetStateFromHash(PancakePuzzleState &node, uint64_t hash);
	static int CountSmallerInRight(const PancakePuzzleState &s, int low, int high);
	static int GetRank(const PancakePuzzleState &s);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
};
/*
class PancakePDB : public PDBHeuristic<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle, PancakePuzzleState, 4> {
public:
	//uint64_t GetStateHash(const RubiksState &s) const;
	//void GetStateFromHash(RubiksState &s, uint64_t hash) const;

	PancakePDB(PancakePuzzle *e, const PancakePuzzleState &s, std::vector<int> distincts);

	virtual uint64_t GetPDBSize() const;

	virtual uint64_t GetPDBHash(const state &s, int threadID = 0) const;
	virtual void GetStateFromPDBHash(uint64_t hash, state &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const state &s, int threadID = 0) const { return GetPDBHash(s); }
	virtual state GetStateFromAbstractState(state &s) const { return s; }


	//	const char *GetName();
	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
private:

	uint64_t Factorial(int val) const;
	uint64_t FactorialUpperK(int n, int k) const;
	std::vector<int> distinct;
	size_t puzzleSize;
	uint64_t pdbSize;
	state example;
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > dualCache;
	mutable std::vector<std::vector<int> > locsCache;
};
*/



#endif