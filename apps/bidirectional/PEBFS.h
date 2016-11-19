#ifndef _PEBFS_H_
#define _PEBFS_H_
#include<stdint.h>//for uint8 types in 

#include <stdio.h>
#include "RubiksCube.h"
#include "IDAStar.h"
#include "Timer.h"
#include <string>
#include <unordered_set>
#include <iomanip>

#ifndef NOT_FOUND
#define NOT_FOUND 10000
#endif

/*RubiksState is replaced by State
since method BuildHeuristics depends on the run-time type of State,
thus we cannot use templates instead of polymorphism
*/
//template <typename state, typename action, typename environment>
//class PEBFS
//{
//
//};

enum  tSearchDirection1 {
	kForward1,
	kBackward1,
};

// This tells us the open list buckets
struct openData1 {
	tSearchDirection1 dir;  // at most 2 bits
	uint8_t priority;      // at most 6 bits
	uint8_t gcost;         // at most 4 bits
	uint8_t hcostF;         // at most 4 bits
	uint8_t hcostB;         // at most 4 bits
	uint8_t bucket;        // at most (3) bits
};

static bool operator==(const openData1& a, const openData1 &b)
{
	return (a.dir == b.dir && a.priority == b.priority && a.gcost == b.gcost && a.hcostF == b.hcostF
		&& a.hcostB == b.hcostB && a.bucket == b.bucket);
}

static std::ostream &operator<<(std::ostream &out, const openData1 &d)
{
	out << "[" << ((d.dir == kForward1) ? "forward" : "backward") << ", p:" << +d.priority << ", g:" << +d.gcost <<
		", hF:" << +d.hcostF << ", hB:" << +d.hcostB;
	out << ", b:" << +d.bucket << "]";
	return out;
}

struct openData1Hash
{
	//std::size_t operator()(const openData1 & x) const
	//{
	//	return (x.dir) | (x.priority << 2) | (x.gcost << 8) | (x.hcost << 12) | (x.bucket << 20);
	//}
	std::size_t operator()(const openData1 & x) const
	{
		std::size_t hash = 0;
		hash = x.bucket;
		hash = hash << 6;
		hash = hash | x.hcostB;
		hash = hash << 6;
		hash = hash | x.hcostF;
		hash = hash << 6;
		hash = hash | x.gcost;
		hash = hash << 6;
		hash = hash | x.priority;
		hash = hash << 2;
		hash = hash | x.dir;
		return hash;
	}
};

//should depth be gcost, in general case?
struct closedData1 {

	tSearchDirection1 dir;
	uint8_t depth;
	uint8_t bucket;
};
static bool operator==(const closedData1 &a, const closedData1 &b)
{
	return (a.dir == b.dir && a.depth == b.depth && a.bucket == b.bucket);
}

static std::ostream &operator<<(std::ostream &out, const closedData1 &d)
{
	out << "[" << ((d.dir == kForward1) ? "forward" : "backward") << ", depth:" << +d.depth;
	out << ", b:" << +d.bucket << "]";
	return out;
}

struct closedData1Hash
{
	//std::size_t operator()(const closedData1 & x) const
	//{
	//	return (x.dir) | (x.bucket << 2) | (x.bucket << 7);
	//}
	std::size_t operator()(const closedData1 & x) const
	{
		std::size_t hash = 0;
		hash = x.bucket;
		hash = hash << 8;
		hash = hash | x.depth;
		hash = hash << 2;
		hash = hash | x.dir;
		return hash;
	}
};

struct openList1 {
	openList1() :f(0) {}
	//std::unordered_set<uint64_t> states;
	FILE *f;
};



struct closedList1 {
	closedList1() :f(0) {}
	FILE *f;
};


//template<class state>
//typedef void (*build_heuristic_function)(state, state, heuristic &);

template<class state,class action, typename heuristic>
class PEBFS
{
public:
	//PEBFS(state &start, state &goal, const char *p1, const char *p2, build_heuristic_function bh_func);
	PEBFS(state &start, state &goal, const char *p1, const char *p2, heuristic& f, heuristic& b,
		SearchEnvironment<state, action>* se, int _cstar = NOT_FOUND);

	void FindAPath();

	//i changed them to be public due to a compiling error
	const int fileBuckets = 32; // must be at least 8
	const uint64_t bucketBits = 5;
	const int bucketMask = 0x1F;
	//~PEBFS();
protected:
	void ExpandNextFile(openData1& d);
	void ExpandNextPair();
	int GetBestPair(openData1& df, openData1& db);
	bool CanTerminateSearch(std::unordered_map<openData1, openList1, openData1Hash> currentOpen);
	void CheckSolution(std::unordered_map<openData1, openList1, openData1Hash> currentOpen, openData1 d,
		const std::unordered_set<uint64_t> &states);
	void WriteToClosed(std::unordered_set<uint64_t> &states, openData1 d);
	void ParallelExpandBucket(openData1 d, const std::unordered_set<uint64_t> &states, int myThread, int totalThreads);
	void RemoveDuplicates(std::unordered_set<uint64_t> &states, openData1 d);

	void ReadBucket(std::unordered_set<uint64_t> &states, openData1 d);

	std::string GetOpenName(const openData1 &d);
	std::string GetClosedName(closedData1 d);

	void AddStatesToQueue(const openData1 &d, uint64_t *data, size_t count);
	void AddStateToQueue(openData1 &d, uint64_t data);
	void AddStateToQueue(const state &start, tSearchDirection1 dir, int cost);
	void GetopenData1(const state &start, tSearchDirection1 dir, int cost,
		openData1 &d, uint64_t &data);

	virtual void GetState(state &s, int bucket, uint64_t data) = 0;
	virtual int GetBucket(const state &s) = 0;
	virtual void GetBucketAndData(const state &s, int &bucket, uint64_t &data) = 0;


	//const int fileBuckets = 32; // must be at least 8
	//const uint64_t bucketBits = 5;
	//const int bucketMask = 0x1F;

	const char *prefix1;
	const char *prefix2;

	bool finished = false;

	int bestSolution;


	uint64_t expanded;
	uint64_t expandedOnFirstSolution;
	std::vector<uint64_t> gDistForward;
	std::vector<uint64_t> gDistBackward;
	//the nodes less than c star distribution
	std::vector<uint64_t> gltcDistForward;
	std::vector<uint64_t> gltcDistBackward;
	std::mutex printLock;
	std::mutex countLock;
	std::mutex openLock;
	heuristic forward;
	heuristic reverse;

	std::unordered_map<closedData1, closedList1, closedData1Hash> closed;
	std::unordered_map<openData1, openList1, openData1Hash> open;

	//cube is not supposed to be here! but just keep it for a moment
	//RubiksCube cube;
	//RubiksCube is a derived class of searchEnvironment<,>
	SearchEnvironment<state, action>* moreThanCube;

	state s;
	state g;


	//optimal solution
	int cstar;


};






//
//  MM.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/31/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//


template<class state,class action, typename heuristic>
PEBFS<state, action, heuristic>::PEBFS(state &start, state &goal, const char *p1, const char *p2, heuristic& f, heuristic& b,
	SearchEnvironment<state, action>* se, int _cstar)
	:prefix1(p1), prefix2(p2), bestSolution(NOT_FOUND), expanded(0), expandedOnFirstSolution(0), moreThanCube(se),cstar(_cstar)

{

	gDistBackward.resize(120);
	gDistForward.resize(120);
	gltcDistBackward.resize(120);
	gltcDistForward.resize(120);
	//(*bh_func)(start, goal, forward);
	//(*bh_func)(goal, start, reverse);

	forward = f;
	reverse = b;
	//moreThanCube = se;
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout << std::setprecision(2);


	s = start;
	g = goal;

}



template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::FindAPath()
{
	Timer t;
	t.StartTimer();
	printf("---PEBFS*---\n");
	AddStateToQueue(s, kForward1, 0);
	AddStateToQueue(g, kBackward1, 0);
	//printf("states added to queue\n");
	while (!open.empty() && !finished)
	{
		ExpandNextPair();
	}
	t.EndTimer();
	printf("%1.2fs elapsed\n", t.GetElapsedTime());
}


template<class state,class action, typename heuristic>
std::string PEBFS<state, action, heuristic>::GetClosedName(closedData1 d)
{
	std::string s;
	if (d.dir == kForward1)
	{
		s += prefix1;
		s += "forward-";
	}
	else {
		s += prefix2;
		s += "backward-";
	}
	s += std::to_string(d.bucket);
	s += "-";
	s += std::to_string(d.depth);
	s += ".closed";
	return s;
}

template<class state,class action, typename heuristic>
std::string PEBFS<state, action, heuristic>::GetOpenName(const openData1 &d)
{
	std::string s;
	if (d.dir == kForward1)
	{
		s += prefix1;
		s += "forward-";
	}
	else {
		s += prefix2;
		s += "backward-";
	}
	s += std::to_string(d.priority);
	s += "-";
	s += std::to_string(d.gcost);
	s += "-";
	s += std::to_string(d.hcostF);
	s += "-";
	s += std::to_string(d.hcostB);
	//	s += "-";
	//	s += std::to_string(d.hcost2);
	s += "-";
	s += std::to_string(d.bucket);
	s += ".open";
	return s;
}

template<class state,class action, typename heuristic>
int PEBFS<state, action, heuristic>::GetBestPair(openData1& df, openData1& db)
{

	int minF = 1000;
	// actually do priority here
	openData1 fbest, bbest;
	//return (open.begin())->first;

	for (const auto &s : open)
	{
		if (s.first.dir == kForward1)
		{
			fbest = s.first;
			break;
		}
	}
	for (const auto &s : open)
	{
		if (s.first.dir == kBackward1)
		{
			bbest = s.first;
			break;
		}
	}
	minF = std::max(fbest.gcost + fbest.hcostF, bbest.gcost + bbest.hcostB);
	minF = std::max(minF, fbest.gcost + bbest.gcost);

	int f_s1_s2;
	for (const auto &s1 : open)
	{
		if (s1.first.dir == kBackward1)
			continue;

		if (s1.first.gcost + s1.first.hcostF >= bestSolution)
			continue;

		//s1 is on open_F, g+h<U

		for (const auto &s2 : open)
		{
			if (s2.first.dir == kForward1)
				continue;

			if (s2.first.gcost + s2.first.hcostB >= bestSolution)
				continue;

			//s2 is on open_B

			f_s1_s2 = std::max(s1.first.gcost + s1.first.hcostF, s2.first.gcost + s2.first.hcostB);
			f_s1_s2 = std::max(f_s1_s2, s1.first.gcost + s2.first.gcost);

			if (f_s1_s2 < minF)
			{
				fbest = s1.first;
				bbest = s2.first;
				minF = f_s1_s2;
			}

			else if (f_s1_s2 == minF)
			{
				if (s1.first.gcost + s2.first.gcost < fbest.gcost + bbest.gcost)
				{
					fbest = s1.first;
					bbest = s2.first;
				}
				else if (s1.first.gcost + s2.first.gcost == fbest.gcost + bbest.gcost)
				{
					if (s1.first.gcost < fbest.gcost)
						fbest = s1.first;
					else if (s1.first.gcost == fbest.gcost && s1.first.bucket < fbest.bucket)
						fbest = s1.first;

					if (s2.first.gcost < bbest.gcost)
						bbest = s2.first;
					else if (s2.first.gcost == bbest.gcost && s2.first.bucket < bbest.bucket)
						bbest = s2.first;
				}
			}
		}
	}


	df = fbest;
	db = bbest;

	return minF;
}



template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::GetopenData1(const state &start, tSearchDirection1 dir, int cost,
	openData1 &d, uint64_t &data)
{
	int bucket;
	//uint64_t data;
	GetBucketAndData(start, bucket, data);
	//openData1 d;
	d.dir = dir;
	d.gcost = cost;

	d.hcostF = forward.HCost(start, g);
	d.hcostB = reverse.HCost(start, s);

	d.bucket = bucket;
	if (d.dir == kForward1)
		d.priority = d.gcost + d.hcostF;
	else
		d.priority = d.gcost + d.hcostB;
	//d.priority = 0;
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::AddStatesToQueue(const openData1 &d, uint64_t *data, size_t count)
{
	openLock.lock();
	if (open.find(d) == open.end())
	{
		open[d].f = fopen(GetOpenName(d).c_str(), "w+b");
		if (open[d].f == 0)
		{
			printf("Error opening %s; Aborting!\n", GetOpenName(d).c_str());
			exit(0);
		}
	}
	if (open[d].f == 0)
	{
		printf("Error - file is null!\n");
		exit(0);
	}
	fwrite(data, sizeof(uint64_t), count, open[d].f);
	openLock.unlock();
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::AddStateToQueue(openData1 &d, uint64_t data)
{
	AddStatesToQueue(d, &data, 1);

}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::AddStateToQueue(const state &start, tSearchDirection1 dir, int cost)
{
	openData1 d;
	uint64_t rank;
	GetopenData1(start, dir, cost, d, rank);
	AddStateToQueue(d, rank);
}



template<class state,class action, typename heuristic>
bool PEBFS<state, action, heuristic>::CanTerminateSearch(std::unordered_map<openData1, openList1, openData1Hash> currentOpen)
{

	for (const auto &s : currentOpen)
	{
		if (s.first.dir == kBackward1)
			continue;
		//throw away all nodes f >= U
		if (s.first.gcost + s.first.hcostF >= bestSolution)
			continue;
		for (const auto &s2 : currentOpen)
		{
			if (s2.first.dir == kForward1)
				continue;
			if (s2.first.gcost + s2.first.hcostB >= bestSolution)
				continue;
			//find a pair s, s2
			if (s.first.gcost + s2.first.gcost < bestSolution
				//&&s.first.gcost + s2.first.gcost+s.first.hcostF - s2.first.hcostF< bestSolution
				//&&s.first.gcost + s2.first.gcost + s2.first.hcostB - s.first.hcostB< bestSolution
				)
				return false;
		}

	}
	printf("Solution to return: %d\n", bestSolution);
	printf("Done!\n");
	printf("%llu nodes expanded\n", expanded);
	printf("Forward Distribution:\n");
	for (int x = 0; x < gDistForward.size(); x++)
		if (gDistForward[x] != 0)
			printf("%d\t%llu\n", x, gDistForward[x]);
	printf("Backward Distribution:\n");
	for (int x = 0; x < gDistBackward.size(); x++)
		if (gDistBackward[x] != 0)
			printf("%d\t%llu\n", x, gDistBackward[x]);
	if (cstar < NOT_FOUND)
	{
		printf("Forward Distribution of f<C*:\n");
		for (int x = 0; x < gltcDistForward.size(); x++)
			if (gltcDistForward[x] != 0)
				printf("%d\t%llu\n", x, gltcDistForward[x]);
		printf("Backward Distribution of f<C*:\n");
		for (int x = 0; x < gltcDistBackward.size(); x++)
			if (gltcDistBackward[x] != 0)
				printf("%d\t%llu\n", x, gltcDistBackward[x]);
	}
	finished = true;
	return true;
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::CheckSolution(std::unordered_map<openData1, openList1, openData1Hash> currentOpen, openData1 d,
	const std::unordered_set<uint64_t> &states)
{
	//std::cout<<"\ncheck solution\n";
	//std::cout << "current Open size" << currentOpen.size() << "\n";
	for (const auto &s : currentOpen)
	{
		if (s.first.dir == kForward1 &&s.first.gcost + s.first.hcostF >= bestSolution)
			continue;
		if (s.first.dir == kBackward1 &&s.first.gcost + s.first.hcostB >= bestSolution)
			continue;
		if (s.first.gcost + d.gcost >= bestSolution)
			continue;
		// Opposite direction, same bucket AND same hcostF and same hcostB could be a solution
		if (s.first.dir != d.dir && s.first.bucket == d.bucket
			&&	s.first.hcostF == d.hcostF && s.first.hcostB == d.hcostB
			)
		{
			const size_t bufferSize = 128;
			uint64_t buffer[bufferSize];
			if (s.second.f == 0)
			{
				std::cout << "Error opening " << s.first << "\n";
				exit(0);
			}
			rewind(s.second.f);
			size_t numRead;
			do {
				numRead = fread(buffer, sizeof(uint64_t), bufferSize, s.second.f);
				for (int x = 0; x < numRead; x++)
				{
					if (states.find(buffer[x]) != states.end())
					{
						printLock.lock();
						//s.second.states.find(data) != s.second.states.end()
						printf("\nFound solution gcost1 %d, hcostF1 %d, hcostB1 %d, gcost2 %d, hcostF2 %d, hcostB2 %d\n",
							d.gcost, d.hcostF, d.hcostB, s.first.gcost, s.first.hcostF, s.first.hcostB);
						printf("\nFound solution cost %d+%d=%d\n", d.gcost, s.first.gcost, d.gcost + s.first.gcost);
						//this is the first solution found
						if (bestSolution == NOT_FOUND)
							expandedOnFirstSolution = expanded;
						bestSolution = std::min(d.gcost + s.first.gcost, bestSolution);
						printf("Current best solution: %d\n", bestSolution);
						printLock.unlock();

						if (CanTerminateSearch(open))
							return;
					}
				}
			} while (numRead == bufferSize);
		}
	}
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::ReadBucket(std::unordered_set<uint64_t> &states, openData1 d)
{
	const size_t bufferSize = 128;
	uint64_t buffer[bufferSize];
	rewind(open[d].f);
	size_t numRead;
	do {
		numRead = fread(buffer, sizeof(uint64_t), bufferSize, open[d].f);
		for (int x = 0; x < numRead; x++)
			states.insert(buffer[x]);
	} while (numRead == bufferSize);
	fclose(open[d].f);
	remove(GetOpenName(d).c_str());
	open[d].f = 0;
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::RemoveDuplicates(std::unordered_set<uint64_t> &states, openData1 d)
{
	for (int depth = d.gcost - 2; depth < d.gcost; depth++)
	{
		closedData1 c;
		c.bucket = d.bucket;
		c.depth = depth;
		c.dir = d.dir;

		//closedList1 &cd = closed[c];
		//if (cd.f == 0)
		//	continue;
		auto cdi = closed.find(c);
		if (cdi == closed.end())
			continue;
		closedList1 &cd = closed[c];
		rewind(cd.f);

		const size_t bufferSize = 1024;
		uint64_t buffer[bufferSize];
		rewind(cd.f);
		size_t numRead;
		do {
			numRead = fread(buffer, sizeof(uint64_t), bufferSize, cd.f);
			for (int x = 0; x < numRead; x++)
			{
				auto i = states.find(buffer[x]);
				if (i != states.end())
					states.erase(i);
			}
		} while (numRead == bufferSize);
	}
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::WriteToClosed(std::unordered_set<uint64_t> &states, openData1 d)
{
	closedData1 c;
	c.bucket = d.bucket;
	c.depth = d.gcost;
	c.dir = d.dir;

	closedList1 &cd = closed[c];
	if (cd.f == 0)
	{
		cd.f = fopen(GetClosedName(c).c_str(), "w+b");
	}
	for (const auto &i : states)
	{
		fwrite(&i, sizeof(uint64_t), 1, cd.f);
	}
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::ParallelExpandBucket(openData1 d, const std::unordered_set<uint64_t> &states, int myThread, int totalThreads)
{
	//std::cout << "\nparallel expanding\n";
	const int cacheSize = 1024;
	std::unordered_map<openData1, std::vector<uint64_t>, openData1Hash> cache;
	state tmp = g;
	uint64_t localExpanded = 0;
	int count = 0;
	//std::cout << "states.size: " << states.size() << "\n";
	for (const auto &values : states)
	{
		//std::cout << "values: " << values << "\n";
		if (finished)
			break;

		count++;
		if (myThread != (count%totalThreads))
			continue;

		localExpanded++;


		//for (int x = 0; x < 18; x++)
		//Replaced by GetActions
		GetState(tmp, d.bucket, values);

		//openData1 dat;
		//uint64_t rak;
		//GetopenData1(tmp, d.dir, d.gcost, dat, rak);

		std::vector<action> actions;
		moreThanCube->GetActions(tmp, actions);
		//std::cout << "get actions\n";
		for (auto x = actions.begin(); x != actions.end(); x++)
		{
			GetState(tmp, d.bucket, values);
			//std::cout << "tmp" << tmp << "\n";
			// copying 2 64-bit values is faster than undoing a move
			moreThanCube->ApplyAction(tmp, *x);
			openData1 newData;
			uint64_t newRank;
			GetopenData1(tmp, d.dir, d.gcost + 1, newData, newRank);

			//if (dat.hcost-newData.hcost > 1)
			//	std::cout << "inconsistency!!!\n";

			std::vector<uint64_t> &c = cache[newData];
			c.push_back(newRank);
			if (c.size() > cacheSize)
			{
				AddStatesToQueue(newData, &c[0], c.size());
				c.clear();
			}
		}
	}
	for (auto &i : cache)
	{
		AddStatesToQueue(i.first, &(i.second[0]), i.second.size());
	}

	countLock.lock();
	expanded += localExpanded;
	if (d.dir == kForward1)
	{
		gDistForward[d.gcost] += localExpanded;
		if (d.gcost + d.hcostF<cstar)
			gltcDistForward[d.gcost] += localExpanded;

	}
	else
	{
		gDistBackward[d.gcost] += localExpanded;
		if (d.gcost + d.hcostB<cstar)
			gltcDistBackward[d.gcost] += localExpanded;
	}
	countLock.unlock();
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::ExpandNextFile(openData1& d)
{
	// 1. Get next expansion target

	Timer timer;
	timer.StartTimer();

	//std::cout << "\ncurrent best d: " << d << "\n";
	for (int depth = 0; depth < d.gcost - 2; depth++)
	{
		closedData1 c;
		c.bucket = d.bucket;
		c.depth = depth;
		c.dir = d.dir;

		auto cdi = closed.find(c);
		while (cdi != closed.end())
		{
			//std::cout << "permanat close: " << cdi->first << "\n";
			fclose(cdi->second.f);
			cdi->second.f = 0;
			closed.erase(cdi);
			cdi = closed.find(c);
		}
	}


	std::unordered_set<uint64_t> states;
	ReadBucket(states, d);
	RemoveDuplicates(states, d);
	WriteToClosed(states, d);
	timer.EndTimer();

	printLock.lock();
	//std::cout << "Next: " << d << " (" << states.size() << " entries) [" << timer.GetElapsedTime() << "s reading/dd] ";
	printLock.unlock();

	timer.StartTimer();
	// Read in opposite buckets to check for solutions in parallel to expanding this bucket
	openLock.lock();
	std::thread t(&PEBFS<state, action, heuristic>::CheckSolution, this, open, d, std::ref(states));
	openLock.unlock();

	// 3. expand all states in current bucket & write out successors
	const int numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread *> threads;
	for (int x = 0; x < numThreads; x++)
		threads.push_back(new std::thread(&PEBFS<state, action, heuristic>::ParallelExpandBucket, this, d, std::ref(states), x, numThreads));
	for (int x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
	open.erase(open.find(d));
	timer.EndTimer();
	printLock.lock();
	//std::cout << "[" << timer.GetElapsedTime() << "s expanding]+";
	timer.StartTimer();
	printLock.unlock();
	t.join();
	printLock.lock();
	timer.EndTimer();
	//std::cout << "[" << timer.GetElapsedTime() << "s]\n";
	printLock.unlock();
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::ExpandNextPair()
{
	// 1. Get next expansion target
	openData1 d1, d2;
	int f = GetBestPair(d1, d2);
	
	//printf("current f: %d\n", f);

	if (CanTerminateSearch(open))
		return;

	ExpandNextFile(d1);

	if (CanTerminateSearch(open))
		return;

	ExpandNextFile(d2);


}

/*
template<class state,class action, typename heuristic>
int PEBFS<state, action, heuristic>::GetBucket(const state &s)
{
//uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
//return ehash&bucketMask;
return 0;
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::GetBucketAndData(const state &s, int &bucket, uint64_t &data)
{
//uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
//uint64_t chash = RubikCornerPDB::GetStateHash(s.corner);
//bucket = ehash&bucketMask;
//data = (ehash >> bucketBits)*RubikCornerPDB::GetStateSpaceSize() + chash;
}

template<class state,class action, typename heuristic>
void PEBFS<state, action, heuristic>::GetState(state &s, int bucket, uint64_t data)
{
//RubikCornerPDB::GetStateFromHash(s.corner, data%RubikCornerPDB::GetStateSpaceSize());
//RubikEdgePDB::GetStateFromHash(s.edge, bucket | ((data / RubikCornerPDB::GetStateSpaceSize()) << bucketBits));
}
*/




#endif // !_PEBFS_H_


