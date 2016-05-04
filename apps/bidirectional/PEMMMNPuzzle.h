#ifndef PEMM_MNPUZZLE_H
#define PEMM_MNPUZZLE_H
#include "PEMM.h"
#include "MNPuzzle.h"
#include "PermutationPDB.h"


uint64_t Factorial(int val);

template<typename heuristic>
class PEMMMNPuzzle : public PEMM<MNPuzzleState, slideDir, heuristic> {
public:
	PEMMMNPuzzle(MNPuzzleState &start, MNPuzzleState &goal, const char *p1, const char *p2, heuristic& f, heuristic& b, MNPuzzle* se)
		: PEMM<MNPuzzleState, slideDir,heuristic>(start, goal, p1, p2, f, b, se)
	{
	}

protected:
	virtual void GetState(MNPuzzleState &s, int bucket, uint64_t data);
	virtual int GetBucket(const MNPuzzleState &s);
	virtual void GetBucketAndData(const MNPuzzleState &s, int &bucket, uint64_t &data);
};
//
//
class MNPuzzlePDB : public PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> {
public:
	MNPuzzlePDB(MNPuzzle* e, const MNPuzzleState& s, std::vector<int> distincts) :
		PermutationPDB(e, s, distincts)
	{
		SetGoal(s);
	}

	static uint64_t GetStateHash(const MNPuzzleState& s);
	static void GetStateFromHash(MNPuzzleState &node, uint64_t hash);
	static uint64_t CountSmallerInRight(const MNPuzzleState &s, int low, int high);
	static uint64_t GetRank(const MNPuzzleState &s);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	//std::string GetFileName(const char *prefix);
};

class ManhattanDistanceHeuristic
{
public:
	ManhattanDistanceHeuristic() { }
	~ManhattanDistanceHeuristic() {}
	void SetGoal(const MNPuzzleState& s) { goal = s; }
	int GetHCost(const MNPuzzleState& s);
	int HCost(const MNPuzzleState& s1, const MNPuzzleState& s2);
	MNPuzzleState goal;
};


class SlidingTilePuzzle
{
public:
	SlidingTilePuzzle() {  }
	SlidingTilePuzzle(unsigned int w, unsigned int h) :width(w), height(h) {  }
	SlidingTilePuzzle(const SlidingTilePuzzle& s)
		:width(s.width), height(s.height)
	{}
	~SlidingTilePuzzle() {}

	void GetRankFromState(const MNPuzzleState& state, uint64_t& rank);
	void GetStateFromRank(MNPuzzleState& state, const uint64_t& rank);

	unsigned int width;
	unsigned int height;

};





template<typename heuristic>
int PEMMMNPuzzle<heuristic>::GetBucket(const MNPuzzleState &s)
{
	//Puzzle puzzle(s.width, s.height);
	//uint64_t hash = puzzle.GetStateHash(s);
	//std::cout << "state: " << s << " bucket: " << (int)(hash & 0x1F) <<"\n";
	SlidingTilePuzzle puzzle(s.width, s.height);
	uint64_t hash;
	puzzle.GetRankFromState(s,hash);
	return hash & 0x3;
}

template<typename heuristic>
void PEMMMNPuzzle<heuristic>::GetBucketAndData(const MNPuzzleState &s, int &bucket, uint64_t &data)
{
	//MNPuzzle puzzle(s.width, s.height);
	//uint64_t hash = puzzle.GetStateHash(s);
	SlidingTilePuzzle puzzle(s.width, s.height);
	uint64_t hash;
	puzzle.GetRankFromState(s, hash);
	bucket = hash & 0x3;
	data = hash >> 2;
	//std::cout << "state: " << s << " bucket: " << bucket << " data: " << data << "\n";

}

template<typename heuristic>
void PEMMMNPuzzle<heuristic>::GetState(MNPuzzleState &s, int bucket, uint64_t data)
{
	//std::cout << "\nget state from data:"<<data <<" bucket:" <<bucket;
	uint64_t hash = (data << 2) | bucket;
	//std::cout << " hash :" << hash << "\n";
	//MNPuzzle puzzle(s.width, s.height);
	//puzzle.GetStateFromHash(s, hash);
	SlidingTilePuzzle puzzle(s.width, s.height);
	puzzle.GetStateFromRank(s, hash);
	//std::cout << "state: " << s << " bucket: " << bucket << " data: " << data << "\n";
}

#endif