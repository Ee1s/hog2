#include "PEMMPancake.h"

int PEMMPancake::GetBucket(const PancakePuzzleState &s)
{
	//return 0;
	//std::cout << "\nget bucket\n";
	uint64_t hash = PancakePDB::GetStateHash(s);
	//std::cout << "state: " << s << " bucket: " << (int)(hash & 0x1F) <<"\n";
	return hash & 0x1F;
	//return ehash & PEMM<RubiksState, RubiksAction>::bucketMask;
}

void PEMMPancake::GetBucketAndData(const PancakePuzzleState &s, int &bucket, uint64_t &data)
{
	//bucket = 0;
	//data = 0;
	//PancakePuzzle pancake(10);
	//data = pancake.GetStateHash(s);
	//4 bits bucket
	//rest for the data
	//uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
	//uint64_t chash = RubikCornerPDB::GetStateHash(s.corner);
	//bucket = ehash & PEMM<RubiksState, RubiksAction>::bucketMask;
	//data = (ehash >> PEMM<RubiksState, RubiksAction>::bucketBits)*RubikCornerPDB::GetStateSpaceSize() + chash;
	//std::cout << "\nget bucket and data\n";
	uint64_t hash = PancakePDB::GetStateHash(s);
	bucket =  hash & 0x1F;
	data = hash >> 5;
	//std::cout << "state: " << s << " bucket: " << bucket << " data: " << data << "\n";

}

void PEMMPancake::GetState(PancakePuzzleState &s, int bucket, uint64_t data)
{
#pragma warning Size not being set correctly
	//PancakePuzzle pancake(10);
	//s.puzzle.resize(10);
	//pancake.GetStateFromHash(s, data);
	//RubikCornerPDB::GetStateFromHash(s.corner, data%RubikCornerPDB::GetStateSpaceSize());
	//RubikEdgePDB::GetStateFromHash(s.edge, bucket | ((data / RubikCornerPDB::GetStateSpaceSize()) << PEMM<RubiksState, RubiksAction>::bucketBits));
	//std::cout << "\nget state from data:"<<data <<" bucket:" <<bucket;
	//std::cout << "fk!!!!" << (int)((0 << 5) | 1) << "\n";
	uint64_t hash = (data << 5) | bucket;
	//std::cout << " hash :" << hash << "\n";
	PancakePDB::GetStateFromHash(s, hash);
	//std::cout << "state: " << s << " bucket: " << bucket << " data: " << data << "\n";
}

bool PancakePDB::Load(const char *prefix)
{

}
void PancakePDB::Save(const char *prefix)
{

}
bool PancakePDB::Load(FILE *f)
{

}
void PancakePDB::Save(FILE *f)
{

}
std::string PancakePDB::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	return fileName;
}

int PancakePDB::CountSmallerInRight(const PancakePuzzleState &s, int low, int high)
{
	int count = 0;

	for (int i = low + 1; i <= high; ++i)
		if (s.puzzle[i] < s.puzzle[low])
			count++;

	return count;
}

uint64_t PancakePDB::GetRank(const PancakePuzzleState &s)
{
	int size = s.puzzle.size();
	uint64_t mul = 0;
	uint64_t rank = 1;
	uint64_t countRight;

	for (int i = 0; i < size; i++)
	{
		mul = Factorial(size-1-i);

		// count number of elementss smaller than puzzle[i]
		// fron puzzle[i+1] to puzzle[size-1]
		countRight = CountSmallerInRight(s, i, size - 1);

		rank += countRight * mul;
	}
	//std::cout << "\npuzzle: " << s << " rank: " << rank << "\n";
	return rank;
}

uint64_t PancakePDB::GetStateHash(const PancakePuzzleState &node)
{
	uint64_t hashVal = 0;
	//for (int x = 0; x < node.puzzle.size(); x++)
	//{
	//	hashVal = (hashVal << 1) + node.puzzle[x];
	//}
	//hashVal = hashVal*Factorial(12) + GetRank(node);
	hashVal = GetRank(node);
	return hashVal;
}

void PancakePDB::GetStateFromHash(PancakePuzzleState &node, uint64_t hash)
{
	int size = node.puzzle.size();
	std::vector<int> pz;
	for (int i = 0; i < size; i++)
		pz.push_back(i);
	uint64_t countRight = 0;
	hash = hash - 1;
	//std::cout << "\n get state from hash, hash:" << hash << "\n";
	for (int i = 0; i < size; i++) 
	{

		countRight = hash / Factorial(size - 1 - i);
		hash = hash%Factorial(size - 1 - i);
		//printf("\ni %d, countRight %d", i, countRight);
		node.puzzle[i] = pz[countRight];
		pz.erase(pz.begin() + countRight);
	}
	//std::cout << "node" << node << "\n";
}