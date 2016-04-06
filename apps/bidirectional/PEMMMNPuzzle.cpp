#include "PEMMMNPuzzle.h"

int PEMMMNPuzzle::GetBucket(const MNPuzzleState &s)
{
	//std::cout << "\nget bucket\n";
	uint64_t hash = MNPuzzlePDB::GetStateHash(s);
	//std::cout << "state: " << s << " bucket: " << (int)(hash & 0x1F) <<"\n";
	return hash & 0x1F;
}

void PEMMMNPuzzle::GetBucketAndData(const MNPuzzleState &s, int &bucket, uint64_t &data)
{
	uint64_t hash = MNPuzzlePDB::GetStateHash(s);
	bucket = hash & 0x1F;
	data = hash >> 5;
	//std::cout << "state: " << s << " bucket: " << bucket << " data: " << data << "\n";

}

void PEMMMNPuzzle::GetState(MNPuzzleState &s, int bucket, uint64_t data)
{
	//std::cout << "\nget state from data:"<<data <<" bucket:" <<bucket;
	uint64_t hash = (data << 5) | bucket;
	//std::cout << " hash :" << hash << "\n";
	MNPuzzlePDB::GetStateFromHash(s, hash);
	//std::cout << "state: " << s << " bucket: " << bucket << " data: " << data << "\n";
}


//
bool MNPuzzlePDB::Load(const char *prefix)
{

}
void MNPuzzlePDB::Save(const char *prefix)
{

}
bool MNPuzzlePDB::Load(FILE *f)
{

}
void MNPuzzlePDB::Save(FILE *f)
{

}
std::string MNPuzzlePDB::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += prefix;
	return fileName;
}
//


//uint64_t MNPuzzlePDB::Factorial(int val)
//{
//	static uint64_t table[21] =
//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
//	if (val > 20)
//		return (uint64_t)-1;
//	return table[val];
//}

uint64_t MNPuzzlePDB::CountSmallerInRight(const MNPuzzleState &s, int low, int high)
{
	uint64_t count = 0;

	for (int i = low + 1; i <= high; ++i)
		if (s.puzzle[i] < s.puzzle[low])
			count++;

	return count;
}

uint64_t MNPuzzlePDB::GetRank(const MNPuzzleState &s)
{
	int size = s.puzzle.size();
	uint64_t mul = 0;
	uint64_t rank = 1;
	uint64_t countRight;
	for (int i = 0; i < size; i++)
	{
		mul = Factorial(size - 1 - i);
		countRight = CountSmallerInRight(s, i, size - 1);
		rank += countRight * mul;
	}

	return rank;
}

uint64_t MNPuzzlePDB::GetStateHash(const MNPuzzleState &node)
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

void MNPuzzlePDB::GetStateFromHash(MNPuzzleState &node, uint64_t hash)
{
	int size = node.puzzle.size();
	std::vector<int> pz;
	for (int i = 0; i < size; i++)
		pz.push_back(i);
	uint64_t countRight = 0;
	hash = hash - 1;
	for (int i = 0; i < size; i++)
	{
		countRight = hash / Factorial(size - 1 - i);
		hash = hash%Factorial(size - 1 - i);
		node.puzzle[i] = pz[countRight];
		pz.erase(pz.begin() + countRight);
	}

	for (int i = 0; i < node.puzzle.size(); i++)
	{
		if (node.puzzle[i] == 0)
		{
			node.blank = i;
			return;
		}
	}
	//std::cout << "node" << node << "\n";
}
