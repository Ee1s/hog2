#ifndef PEMM_MNPUZZLE_H
#define PEMM_MNPUZZLE_H
#include "PEMM.h"
#include "MNPuzzle.h"
#include "PermutationPDB.h"

class PEMMMNPuzzle : public PEMM<MNPuzzleState, slideDir> {
public:
	PEMMMNPuzzle(MNPuzzleState &start, MNPuzzleState &goal, const char *p1, const char *p2, Heuristic<MNPuzzleState>& f, Heuristic<MNPuzzleState>& b, MNPuzzle* se)
		: PEMM<MNPuzzleState, slideDir>(start, goal, p1, p2, f, b, se)
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
		//goalState = s;
	}

	static uint64_t GetStateHash(const MNPuzzleState& s);
	static void GetStateFromHash(MNPuzzleState &node, uint64_t hash);
	static uint64_t CountSmallerInRight(const MNPuzzleState &s, int low, int high);
	static uint64_t GetRank(const MNPuzzleState &s);
	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
};
//

/*
namespace MNPuzzlePDB {
	uint64_t Factorial(int val);
	//{
	//	static uint64_t table[21] =
	//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
	//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
	//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	//	if (val > 20)
	//		return (uint64_t)-1;
	//	return table[val];
	//}

	int CountSmallerInRight(const MNPuzzleState &s, int low, int high);
	//{
	//	int count = 0;
	//
	//	for (int i = low + 1; i <= high; ++i)
	//		if (s.puzzle[i] < s.puzzle[low])
	//			count++;
	//
	//	return count;
	//}
	
	int GetRank(const MNPuzzleState &s);
	//{
	//	int size = s.puzzle.size();
	//	int mul = Factorial(size);
	//	int rank = 1;
	//	int countRight;
	//
	//	for (int i = 0; i < size; i++)
	//	{
	//		mul = Factorial(size - 1 - i);
	//
	//		// count number of chars smaller than str[i]
	//		// fron str[i+1] to str[len-1]
	//		countRight = CountSmallerInRight(s, i, size - 1);
	//
	//		rank += countRight * mul;
	//	}
	//
	//	return rank;
	//}
	
	uint64_t GetStateHash(const MNPuzzleState &node);
	//{
	//	uint64_t hashVal = 0;
	//	//for (int x = 0; x < node.puzzle.size(); x++)
	//	//{
	//	//	hashVal = (hashVal << 1) + node.puzzle[x];
	//	//}
	//	//hashVal = hashVal*Factorial(12) + GetRank(node);
	//	hashVal = GetRank(node);
	//	return hashVal;
	//}
	
	void GetStateFromHash(MNPuzzleState &node, uint64_t hash);
	//{
	//	int size = node.puzzle.size();
	//	std::vector<int> pz;
	//	for (int i = 0; i < size; i++)
	//		pz.push_back(i);
	//	int countRight = 0;
	//	hash = hash - 1;
	//	//std::cout << "\n get state from hash, hash:" << hash << "\n";
	//	for (int i = 0; i < size; i++)
	//	{
	//		countRight = hash / Factorial(size - 1 - i);
	//		hash = hash%Factorial(size - 1 - i);
	//		node.puzzle[i] = pz[countRight];
	//		pz.erase(pz.begin() + countRight);
	//	}
	//	//std::cout << "node" << node << "\n";
	//}
}
*/

#endif