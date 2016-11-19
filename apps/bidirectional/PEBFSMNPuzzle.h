#ifndef PEBFS_MNPUZZLE_H
#define PEBFS_MNPUZZLE_H
#include "PEBFS.h"
#include "MNPuzzle.h"
#include "PermutationPDB.h"
#include "PEMMMNPuzzle.h"



template<typename heuristic>
class PEBFSMNPuzzle : public PEBFS<MNPuzzleState, slideDir, heuristic> {
public:
	PEBFSMNPuzzle(MNPuzzleState &start, MNPuzzleState &goal, const char *p1, const char *p2, heuristic& f, heuristic& b, MNPuzzle* se,
		 int cstar = NOT_FOUND)
		: PEBFS<MNPuzzleState, slideDir, heuristic>(start, goal, p1, p2, f, b, se, cstar)
	{
	}

protected:
	virtual void GetState(MNPuzzleState &s, int bucket, uint64_t data);
	virtual int GetBucket(const MNPuzzleState &s);
	virtual void GetBucketAndData(const MNPuzzleState &s, int &bucket, uint64_t &data);
};

template<typename heuristic>
int PEBFSMNPuzzle<heuristic>::GetBucket(const MNPuzzleState &s)
{
	//Puzzle puzzle(s.width, s.height);
	//uint64_t hash = puzzle.GetStateHash(s);
	//std::cout << "state: " << s << " bucket: " << (int)(hash & 0x1F) <<"\n";
	SlidingTilePuzzle puzzle(s.width, s.height);
	uint64_t hash;
	puzzle.GetRankFromState(s, hash);
	return hash & 0x3;
}

template<typename heuristic>
void PEBFSMNPuzzle<heuristic>::GetBucketAndData(const MNPuzzleState &s, int &bucket, uint64_t &data)
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
void PEBFSMNPuzzle<heuristic>::GetState(MNPuzzleState &s, int bucket, uint64_t data)
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