#ifndef _PEMM_H_
#define _PEMM_H_
#include<stdint.h>//for uint8 types in 

#include <stdio.h>
#include "RubiksCube.h"
#include "IDAStar.h"
#include "Timer.h"
#include <string>
#include <unordered_set>
#include <iomanip>

#define NOT_FOUND 1000000

/*RubiksState is replaced by State
since method BuildHeuristics depends on the run-time type of State,
thus we cannot use templates instead of polymorphism
*/
//template <typename state, typename action, typename environment>
//class PEMM
//{
//
//};

enum  tSearchDirection {
	kForward,
	kBackward,
};

// This tells us the open list buckets
struct openData {
	tSearchDirection dir;  // at most 2 bits
	uint8_t priority;      // at most 6 bits
	uint8_t gcost;         // at most 4 bits
	uint8_t hcost;         // at most 4 bits
	uint8_t bucket;        // at most (3) bits
};

static bool operator==(const openData& a, const openData &b)
	{
		return (a.dir == b.dir && a.priority == b.priority && a.gcost == b.gcost && a.hcost == b.hcost && a.bucket == b.bucket);
	}

static std::ostream &operator<<(std::ostream &out, const openData &d)
	{
		out << "[" << ((d.dir == kForward) ? "forward" : "backward") << ", p:" << +d.priority << ", g:" << +d.gcost << ", h:" << +d.hcost;
		out << ", b:" << +d.bucket << "]";
		return out;
	}

struct openDataHash
{
	//std::size_t operator()(const openData & x) const
	//{
	//	return (x.dir) | (x.priority << 2) | (x.gcost << 8) | (x.hcost << 12) | (x.bucket << 20);
	//}
	std::size_t operator()(const openData & x) const
	{
		std::size_t hash = 0;
		hash = x.bucket;
		hash = hash << 8;
		hash = hash | x.hcost;
		hash = hash << 8;
		hash = hash | x.gcost;
		hash = hash << 8;
		hash = hash | x.priority;
		hash = hash << 2;
		hash = hash | x.dir;
		return hash;
	}
};

//should depth be gcost, in general case?
struct closedData {

	tSearchDirection dir;
	uint8_t depth;
	uint8_t bucket;
};
static bool operator==(const closedData &a,const closedData &b)
	{
		return (a.dir == b.dir && a.depth == b.depth && a.bucket == b.bucket);
	}

static std::ostream &operator<<(std::ostream &out, const closedData &d)
{
	out << "[" << ((d.dir == kForward) ? "forward" : "backward") << ", depth:" << +d.depth;
	out << ", b:" << +d.bucket << "]";
	return out;
}

struct closedDataHash
{
	//std::size_t operator()(const closedData & x) const
	//{
	//	return (x.dir) | (x.bucket << 2) | (x.bucket << 7);
	//}
	std::size_t operator()(const closedData & x) const
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

struct openList {
	openList() :f(0) {}
	//std::unordered_set<uint64_t> states;
	FILE *f;
};



struct closedList {
	closedList() :f(0) {}
	FILE *f;
};


//template<class state>
//typedef void (*build_heuristic_function)(state, state, Heuristic<state> &);

template<class state,class action, typename heuristic>
class PEMM
{
public:
	//PEMM(state &start, state &goal, const char *p1, const char *p2, build_heuristic_function bh_func);
	PEMM(state &start, state &goal, const char *p1, const char *p2, heuristic& f, heuristic& b, 
		SearchEnvironment<state, action>* se,double _lambda=2.0, int _dirs=2, int _cstar =NOT_FOUND, int _aaf=2);
		
	void FindAPath();

	//i changed them to be public due to a compiling error
	const int fileBuckets = 32; // must be at least 8
	const uint64_t bucketBits = 5;
	const int bucketMask = 0x1F;

		//~PEMM();
protected:
	void ExpandNextFile();
	openData GetBestFile();
	bool CanTerminateSearch();
	void CheckSolution(std::unordered_map<openData, openList, openDataHash> currentOpen, openData d,
		const std::unordered_set<uint64_t> &states);
	void WriteToClosed(std::unordered_set<uint64_t> &states, openData d);
	void ParallelExpandBucket(openData d, const std::unordered_set<uint64_t> &states, int myThread, int totalThreads);
	void RemoveDuplicates(std::unordered_set<uint64_t> &states, openData d);

	void ReadBucket(std::unordered_set<uint64_t> &states, openData d);

	std::string GetOpenName(const openData &d);
	std::string GetClosedName(closedData d);

	void AddStatesToQueue(const openData &d, uint64_t *data, size_t count);
	void AddStateToQueue(openData &d, uint64_t data);
	void AddStateToQueue(const state &start, tSearchDirection dir, int cost);
	void GetOpenData(const state &start, tSearchDirection dir, int cost,
		openData &d, uint64_t &data);

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
	int currentC;
	int minGForward;
	int minGBackward;
	int minFForward;
	int minFBackward;
	uint64_t expanded;
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

	std::unordered_map<closedData, closedList, closedDataHash> closed;
	std::unordered_map<openData, openList, openDataHash> open;

	//cube is not supposed to be here! but just keep it for a moment
	//RubiksCube cube;
	//RubiksCube is a derived class of searchEnvironment<,>
	SearchEnvironment<state, action>* moreThanCube;

	double lambda;
	uint64_t expandedOnFirstSolution;
	state s;
	state g;

	//dirs ==2 means bidirectional, dirs ==0 means pure forward search, dirs ==1 means pure backward search
	int dirs;

	//optimal solution
	int cstar;

	int sol_g;
	tSearchDirection sol_dir;
	int actionAfterFound;
};






//
//  MM.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/31/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//


template<class state, class action, typename heuristic>
PEMM<state, action, heuristic>::PEMM(state &start, state &goal, const char *p1, const char *p2, heuristic& f, heuristic& b,
	SearchEnvironment<state, action>* se, double _lambda, int _dirs, int _cstar, int _aaf)
	:prefix1(p1), prefix2(p2), bestSolution(NOT_FOUND), expanded(0), forward(f), reverse(b),
	lambda(_lambda), dirs(_dirs), cstar(_cstar), actionAfterFound(_aaf)
	{
		std::cout << "aaf: "<< actionAfterFound <<"\n";
	gDistBackward.resize(120);
	gDistForward.resize(120);
	gltcDistBackward.resize(120);
	gltcDistForward.resize(120);

	//(*bh_func)(start, goal, forward);
	//(*bh_func)(goal, start, reverse);


	moreThanCube = se;
	std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
	std::cout << std::setprecision(2);

	//	printf("---IDA*---\n");
	//	std::vector<RubiksAction> path;
	//	Timer t;
	//	t.StartTimer();
	//	cube.SetPruneSuccessors(true);
	//	IDAStar<RubiksState, RubiksAction> ida;
	//	ida.SetHeuristic(&forward);
	//	ida.GetPath(&cube, start, goal, path);
	//	t.EndTimer();
	//	printf("%1.5fs elapsed\n", t.GetElapsedTime());
	//	printf("%llu nodes expanded (%1.3f nodes/sec)\n", ida.GetNodesExpanded(),
	//		   ida.GetNodesExpanded()/t.GetElapsedTime());
	//	printf("%llu nodes generated (%1.3f nodes/sec)\n", ida.GetNodesTouched(),
	//		   ida.GetNodesTouched()/t.GetElapsedTime());
	//	cube.SetPruneSuccessors(false);
	//	printf("Solution cost: %d\n", path.size());
	s = start;
	g = goal;

}



template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::FindAPath()
{
	Timer t;
	t.StartTimer();
	printf("---PEMM*---\n");
	AddStateToQueue(s, kForward, 0);
	AddStateToQueue(g, kBackward, 0);
	//printf("states added to queue\n");
	while (!open.empty() && !finished)
	{
		ExpandNextFile();
	}
	t.EndTimer();
	printf("%1.2fs elapsed\n", t.GetElapsedTime());
}


template<class state, class action, typename heuristic>
std::string PEMM<state, action, heuristic>::GetClosedName(closedData d)
{
	std::string s;
	if (d.dir == kForward)
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

template<class state, class action, typename heuristic>
std::string PEMM<state, action, heuristic>::GetOpenName(const openData &d)
{
	std::string s;
	if (d.dir == kForward)
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
	s += std::to_string(d.hcost);
	//	s += "-";
	//	s += std::to_string(d.hcost2);
	s += "-";
	s += std::to_string(d.bucket);
	s += ".open";
	return s;
}

template<class state, class action, typename heuristic>
openData PEMM<state, action, heuristic>::GetBestFile()
{
	minGForward = 1000;
	minGBackward = 1000;
	minFForward = 1000;
	minFBackward = 1000;
	// actually do priority here
	openData best = (open.begin())->first;
	//return (open.begin())->first;
	for (const auto &s : open)
	{
		if (dirs == 0 && s.first.dir == kForward)
		{
			best = s.first;
			break;
		}

		if (dirs == 1 && s.first.dir == kBackward)
		{
			best = s.first;
			break;
		}
	}
	for (const auto &s : open)
	{
		//dirs == 1 means pure backward search
		if (dirs == 1 && s.first.dir == kForward)
			continue;

		//dirs == 0 means pure forward search
		if (dirs == 0 && s.first.dir == kBackward)
			continue;

		if (actionAfterFound ==2 && 2 * s.first.gcost >= bestSolution)
			continue;
		if (actionAfterFound == 1)
		{
			if(s.first.gcost*1.5 >= bestSolution && s.first.dir==kForward)
				continue;
			if (s.first.gcost*3 >= bestSolution && s.first.dir == kBackward)
				continue;
		} 
		if (actionAfterFound == 0 && bestSolution != NOT_FOUND)
		{
			if (s.first.dir == sol_dir && s.first.gcost >= sol_g)
				continue;

			if (s.first.dir != sol_dir && s.first.gcost > bestSolution - sol_g)
				continue;
		}

		if (s.first.dir == kForward && s.first.gcost < minGForward)
			minGForward = s.first.gcost;
		else if (s.first.dir == kBackward && s.first.gcost < minGBackward)
			minGBackward = s.first.gcost;

		if (s.first.dir == kForward && s.first.gcost + s.first.hcost < minFForward)
			minFForward = s.first.gcost + s.first.hcost;
		else if (s.first.dir == kBackward && s.first.gcost + s.first.hcost < minFBackward)
			minFBackward = s.first.gcost + s.first.hcost;

		if (s.first.priority < best.priority)
		{
			best = s.first;
		}
		else if (s.first.priority == best.priority)
		{
			if (s.first.gcost < best.gcost)
				best = s.first;
			else if (s.first.gcost == best.gcost)
			{
				if (s.first.dir == best.dir)
				{
					if (best.bucket > s.first.bucket)
						best = s.first;
				}
				else if (s.first.dir == kForward)
					best = s.first;
			}
		}
	}
	return best;
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::GetOpenData(const state &start, tSearchDirection dir, int cost,
	openData &d, uint64_t &data)
{

	int bucket;
	//uint64_t data;
	GetBucketAndData(start, bucket, data);
	//openData d;
	d.dir = dir;
	d.gcost = cost;
	d.hcost = (dir == kForward) ? forward.HCost(start, g) : reverse.HCost(start, s);

	//std::cout << "start: " << start << "\n";
	//std::cout << "s: " << s << "\n";
	//std::cout << "g: " << g << "\n";
	//std::cout << "hcost: " << (int)d.hcost << "\n";
	//d.hcost2 = (dir==kForward)?reverse.HCost(start, start):forward.HCost(start, start);
	d.bucket = bucket;
	//d.priority = d.gcost+d.hcost;
	d.priority = std::max(d.gcost + d.hcost, (int)(((double)d.gcost) * lambda));
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::AddStatesToQueue(const openData &d, uint64_t *data, size_t count)
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

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::AddStateToQueue(openData &d, uint64_t data)
{
	AddStatesToQueue(d, &data, 1);
	//	if (open.find(d) == open.end())
	//	{
	//		openLock.lock();
	//		open[d].f = fopen(GetOpenName(d).c_str(), "w+b");
	//		if (open[d].f == 0)
	//		{
	//			printf("Error opening %s; Aborting!\n", GetOpenName(d).c_str());
	//			exit(0);
	//		}
	//		openLock.unlock();
	//	}
	//	if (open[d].f == 0)
	//	{
	//		printf("Error - file is null!\n");
	//		exit(0);
	//	}
	//	fwrite(&data, sizeof(data), 1, open[d].f);
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::AddStateToQueue(const state &start, tSearchDirection dir, int cost)
{
	openData d;
	uint64_t rank;
	GetOpenData(start, dir, cost, d, rank);
	AddStateToQueue(d, rank);
}

//void ReadFile(const openData &d, std::unordered_map<RubiksState, bool> &map)
//{
//	// 1. open file for d;
//	FILE *f = fopen(GetOpenName(d).c_str(), "r");
//	assert(f != 0);
//	uint64_t data;
//	RubiksState s;
//	while (fread(&data, 1, sizeof(uint64_t), f) == 1)
//	{
//		GetState(s, d.bucket, data);
//		map[s] = true;
//	}
//	fclose(f);
//	// keep file for duplicate detection purposes
//	//remove(GetOpenName(d).c_str());
//}

template<class state, class action, typename heuristic>
bool PEMM<state, action, heuristic>::CanTerminateSearch()
{
	int val;
	if (bestSolution <= (val = std::max(currentC, std::max(minFForward, std::max(minFBackward, minGBackward + minGForward + 1)))))
	{
		printf("Done!\n");
		printf("%llu nodes expanded\n", expanded);
		printf("%llu nodes expanded when finding the first solution\n", expandedOnFirstSolution);
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
		if (val == currentC)
			printf("-Triggered by current priority\n");
		if (val == minFForward)
			printf("-Triggered by f in the forward direction\n");
		if (val == minFBackward)
			printf("-Triggered by f in the backward direction\n");
		if (val == minGBackward + minGForward + 1)
			printf("-Triggered by gforward+gbackward+1\n");
		return true;
	}
	return false;
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::CheckSolution(std::unordered_map<openData, openList, openDataHash> currentOpen, openData d,
	const std::unordered_set<uint64_t> &states)
{
	//std::cout<<"\ncheck solution\n";
	//std::cout << "current Open size" << currentOpen.size() << "\n";
	for (const auto &s : currentOpen)
	{
		//std::cout << "d info:" << d<<"\n";
		//std::cout << "s info: s.first" << s.first << "\n";
		// Opposite direction, same bucket AND could be a solution (g+g >= C)
		if (s.first.dir != d.dir && s.first.bucket == d.bucket &&
			d.gcost + s.first.gcost >= currentC)// && d.hcost2 == s.first.hcost)
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
						printf("\nFound solution gcost1 %d hcost1 %d gcost2 %d hcost2 %d\n", d.gcost,d.hcost, s.first.gcost,s.first.hcost);
						printf("\nFound solution cost %d+%d=%d\n", d.gcost, s.first.gcost, d.gcost + s.first.gcost);
						//this is the first solution found
						if (bestSolution == NOT_FOUND)
							expandedOnFirstSolution = expanded;
						if (d.gcost + s.first.gcost < bestSolution)
						{
							bestSolution = d.gcost + s.first.gcost;
							sol_dir = d.dir;
							sol_g = d.gcost;
							//std::cout << "sol_dir: " << sol_dir << " sol_g: " << sol_g << " \n";
						}

						printf("Current best solution: %d\n", bestSolution);
						printLock.unlock();

						if (CanTerminateSearch())
							return;
					}
				}
			} while (numRead == bufferSize);
		}
	}
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::ReadBucket(std::unordered_set<uint64_t> &states, openData d)
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

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::RemoveDuplicates(std::unordered_set<uint64_t> &states, openData d)
{
	for (int depth = d.gcost - 2; depth < d.gcost; depth++)
	{
		closedData c;
		c.bucket = d.bucket;
		c.depth = depth;
		c.dir = d.dir;

		//closedList &cd = closed[c];
		//if (cd.f == 0)
		//	continue;
		auto cdi = closed.find(c);
		if (cdi == closed.end())
			continue;
		closedList &cd = closed[c];
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

	if (bestSolution == NOT_FOUND)
		return;

	for (int depth = bestSolution/2; depth < bestSolution; depth++)
	{
		closedData c;
		c.bucket = d.bucket;
		c.depth = depth;
		c.dir = (d.dir == kForward ? kBackward : kForward);

		//closedList &cd = closed[c];
		//if (cd.f == 0)
		//	continue;
		auto cdi = closed.find(c);
		if (cdi == closed.end())
			continue;
		closedList &cd = closed[c];
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

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::WriteToClosed(std::unordered_set<uint64_t> &states, openData d)
{
	closedData c;
	c.bucket = d.bucket;
	c.depth = d.gcost;
	c.dir = d.dir;

	closedList &cd = closed[c];
	if (cd.f == 0)
	{
		cd.f = fopen(GetClosedName(c).c_str(), "w+b");
	}
	for (const auto &i : states)
	{
		fwrite(&i, sizeof(uint64_t), 1, cd.f);
	}
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::ParallelExpandBucket(openData d, const std::unordered_set<uint64_t> &states, int myThread, int totalThreads)
{
	//std::cout << "\nparallel expanding\n";
	const int cacheSize = 1024;
	std::unordered_map<openData, std::vector<uint64_t>, openDataHash> cache;
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

		//openData dat;
		//uint64_t rak;
		//GetOpenData(tmp, d.dir, d.gcost, dat, rak);

		std::vector<action> actions;
		moreThanCube->GetActions(tmp, actions);
		for (auto x = actions.begin(); x != actions.end(); x++)
		{
			GetState(tmp, d.bucket, values);
			// copying 2 64-bit values is faster than undoing a move
			//std::cout << "state:" << tmp<<"\n";
			//std::cout << "action:" << *x<<"\n";
			moreThanCube->ApplyAction(tmp, *x);
			openData newData;
			uint64_t newRank;
			GetOpenData(tmp, d.dir, d.gcost + 1, newData, newRank);

			//std::cout << "hcost:" << (int)newData.hcost << "\n";

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
	if (d.dir == kForward)
	{
		gDistForward[d.gcost] += localExpanded;
		if(d.gcost + d.hcost<cstar)
			gltcDistForward[d.gcost] += localExpanded;

	}		
	else
	{
		gDistBackward[d.gcost] += localExpanded;
		if (d.gcost + d.hcost<cstar)
			gltcDistBackward[d.gcost] += localExpanded;
	}
	
	countLock.unlock();
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::ExpandNextFile()
{
	// 1. Get next expansion target
	openData d = GetBestFile();
	currentC = d.priority;

	if (CanTerminateSearch())
		return;

	Timer timer;
	timer.StartTimer();

	//std::cout << "\ncurrent best d: " << d << "\n";
	for (int depth = 0; depth < d.gcost - 2; depth++)
	{
		closedData c;
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
	std::thread t(&PEMM<state, action, heuristic>::CheckSolution, this, open, d, std::ref(states));
	openLock.unlock();

	// 3. expand all states in current bucket & write out successors
	const int numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread *> threads;
	for (int x = 0; x < numThreads; x++)
		threads.push_back(new std::thread(&PEMM<state, action, heuristic>::ParallelExpandBucket, this, d, std::ref(states), x, numThreads));
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

/*
template<class state, class action, typename heuristic>
int PEMM<state, action, heuristic>::GetBucket(const state &s)
{
//uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
//return ehash&bucketMask;
return 0;
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::GetBucketAndData(const state &s, int &bucket, uint64_t &data)
{
//uint64_t ehash = RubikEdgePDB::GetStateHash(s.edge);
//uint64_t chash = RubikCornerPDB::GetStateHash(s.corner);
//bucket = ehash&bucketMask;
//data = (ehash >> bucketBits)*RubikCornerPDB::GetStateSpaceSize() + chash;
}

template<class state, class action, typename heuristic>
void PEMM<state, action, heuristic>::GetState(state &s, int bucket, uint64_t data)
{
//RubikCornerPDB::GetStateFromHash(s.corner, data%RubikCornerPDB::GetStateSpaceSize());
//RubikEdgePDB::GetStateFromHash(s.edge, bucket | ((data / RubikCornerPDB::GetStateSpaceSize()) << bucketBits));
}
*/




#endif // !_PEMM_H_


