/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <cassert>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <vector>
#include "GUICode.h"
#include "Timer.h"
#include "RubiksCube.h"
#include "MMRubik.h"
#include "MM0Rubik.h"
#include "PEMM.h"
#include "PEMMRubik.h"
#include "PEMMPancake.h"
#include "PEMMMNPuzzle.h"
#include "MR1PermutationPDB.h"

struct hash128
{
	uint32_t parent;
	uint32_t cornerHash;
	uint64_t edgeHash;
};

bool operator< (const hash128 &left, const hash128 &right)
{
	if (left.cornerHash != right.cornerHash)
		return (left.cornerHash < right.cornerHash);
	return left.edgeHash < right.edgeHash;
}

bool operator==(const hash128 &left, const hash128 &right)
{
	return (left.cornerHash == right.cornerHash && left.edgeHash == right.edgeHash);
}


namespace std {
	template <> struct hash<hash128>
	{
		size_t operator()(const hash128 & x) const
		{
			return x.edgeHash^(uint64_t(x.cornerHash)<<40);
		}
	};
}


void GetInstanceFromStdin(RubiksState &start);
void WriteStatesToDisk(std::vector<hash128> &states, int depth);
void ExpandLayer(int depth);
bool DuplicateDetectLayer(int depthToRemove);
void ClearFiles();
void TestPruning(int depth, int bucket);
const int kNumBuckets = 512;
void BFS();
void GetKorfInstance(RubiksState &start, int which);
void GetSuperFlip(RubiksState &start);
void GetDepth20(RubiksState &start, int which);
void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result);
void BuildHeuristics(int count, PancakePuzzleState start, PancakePuzzleState goal, Heuristic<PancakePuzzleState> &result);
void BuildHeuristics(MNPuzzleState start, MNPuzzleState goal, Heuristic<MNPuzzleState>& result,int size);
void GetInstance(int which, PancakePuzzleState &s);
void GetMNPuzzleInstance(int which, MNPuzzleState &s);

char *hprefix;

#define KORF97

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	// technically we could/should install a command-line handler and handle these there
	if (argc > 1 && strcmp(argv[1], "-bfs") == 0)
	{
		BFS();
	}
	else if (argc > 1 && strcmp(argv[1], "-pancake") == 0)
	{
		//MMPancake::MM();
		hprefix = argv[5];
		PancakePuzzleState start(13);
		PancakePuzzleState goal(13);

		Heuristic<PancakePuzzleState> forward;
		Heuristic<PancakePuzzleState> reverse;
		BuildHeuristics(3, start, goal, forward);
		BuildHeuristics(3, goal, start, reverse);

		PancakePuzzle puzzle(13);
		PEMMPancake *searcher;
		//searcher->FindAPath();
		//for (int instance = 69; instance < 100; instance++)
		//{
		//	GetInstance(instance, start);
		//	goal.Reset();
		//	std::cout << "Start: " << start << std::endl;
		//	std::cout << "Goal: " << goal << std::endl;
		//	searcher = new PEMMPancake(start, goal, argv[3], argv[4], forward, reverse, &puzzle);
		//	searcher->FindAPath();
		//	delete searcher;
		//}
		start.Reset();
		std::vector<PancakePuzzleAction> actions;
		puzzle.GetActions(start, actions);
		puzzle.ApplyAction(start, actions[0]);
		puzzle.ApplyAction(start, actions[1]);
		puzzle.ApplyAction(start, actions[2]);
		puzzle.ApplyAction(start, actions[3]);
		puzzle.ApplyAction(start, actions[4]);
		goal.Reset();
		std::cout << "Start: " << start << std::endl;
		std::cout << "Goal: " << goal << std::endl;
		searcher = new PEMMPancake(start, goal, argv[3], argv[4], forward, reverse, &puzzle);
		searcher->FindAPath();
	}
	else if (strcmp(argv[1], "-pida") == 0)
	{
		if (argc <= 5)
		{
			printf("Usage:\n%s -pida <problem> <tmpdir1 <tmpdir2> <heuristicdir>\n", argv[0]);
			exit(0);
		}
		RubiksState a, b;
		RubiksCube c;
		b.Reset();

		int which = 0;
		which = atoi(argv[2]);
		if (which < 10)
			GetKorfInstance(a, which);
		else if (which == 19)
		{
			GetSuperFlip(a);
			// Any action will reduce this to 19 moves to solve
			c.ApplyAction(a, 0);
		}
		else if (which == 20)
		{
			GetSuperFlip(a);
		}
		MM::CompareIDA(a, b, argv[3], argv[4], argv[5]);
	}
	else if (strcmp(argv[1], "-mm0") == 0)
	{
		if (argc <= 4)
		{
			printf("Usage:\n%s -mm0 <problem> <tmpdir1 <tmpdir2>\n", argv[0]);
			exit(0);
		}
		RubiksState a, b;
		RubiksCube c;
		b.Reset();
		
		int which = 0;
		which = atoi(argv[2]);
		if (which < 10)
			GetKorfInstance(a, which);
		else if (which == 19)
		{
			GetSuperFlip(a);
			// Any action will reduce this to 19 moves to solve
			c.ApplyAction(a, 0);
		}
		else if (which == 20)
		{
			GetSuperFlip(a);
		}
		MM0::MM0(a, b, argv[3], argv[4]);
	}
	else if (strcmp(argv[1], "-mm") == 0)
	{
		if (argc <= 5)
		{
			printf("Usage:\n%s -mm <problem> <tmpdir1 <tmpdir2> <heuristicdir> [lambda] [aaf]\n", argv[0]);
			exit(0);
		}
		RubiksState a, b;
		RubiksCube c;
		b.Reset();
		// solution depth 12
		c.ApplyAction(a, 0*3);
		c.ApplyAction(a, 1*3);
		c.ApplyAction(a, 2*3);
		c.ApplyAction(a, 3*3);
		c.ApplyAction(a, 4*3);
		c.ApplyAction(a, 5*3);
		c.ApplyAction(a, 4*3);
		c.ApplyAction(a, 3*3);
		c.ApplyAction(a, 2*3);
		c.ApplyAction(a, 1*3);
		c.ApplyAction(a, 0*3);
		c.ApplyAction(a, 1*3);
		c.ApplyAction(a, 2*3);
		c.ApplyAction(a, 3*3);
		c.ApplyAction(a, 4*3);
		
		RubiksState arr[10];
		for(int i=0;i<10;i++)
			arr[i].Reset();
		srand(20160622);
		for(int i=0;i<10;i++)
		{
			for(int j=0;j<20;j++)
				c.ApplyAction(arr[i],rand()%18);
			std::cout<<arr[i]<<"\n";
		}

		int which = 0;
		which = atoi(argv[2]);
		if (which < 10)
			GetKorfInstance(a, which);
//		else if (which == 19)
//		{
//			GetSuperFlip(a);
//			// Any action will reduce this to 19 moves to solve
//			c.ApplyAction(a, 0);
//		}
		else if(which <20)
		{
			a = arr[which-10];
		}
		else if (which == 20)
		{
			GetSuperFlip(a);
		}
		else if (which > 20)
		{
			GetDepth20(a, which-21);
		}
		hprefix = argv[5];
		Heuristic<RubiksState> forward;
		Heuristic<RubiksState> reverse;
		BuildHeuristics(a, b, forward);
		BuildHeuristics(b, a, reverse);

		double lambda = 2.0;
		if (argc > 6)
			lambda = atof(argv[6]);
		int aaf = 2;
		if (argc > 7)
			aaf = atoi(argv[7]);
		RubiksCube* cube = new RubiksCube();
		PEMMRubik *searcher = new PEMMRubik(a, b, argv[3], argv[4], forward, reverse, cube,lambda,aaf);

		searcher->FindAPath();

		//MM::MM(a, b, argv[3], argv[4], argv[5]);
	}
	else if (argc > 3 && strcmp(argv[1], "-grid") == 0)
	{
		AnalyzeMap(argv[2], argv[3]);
	}
	else if (argc > 3 && strcmp(argv[1], "-testPruning") == 0)
	{
		TestPruning(atoi(argv[2]), atoi(argv[3]));
	}
	else if (strcmp(argv[1], "-mnpuzzle3") == 0)
	{
		hprefix = argv[5];
		int sz = 3;
		MNPuzzleState start(sz,sz);
		MNPuzzleState goal(sz, sz);

		Heuristic<MNPuzzleState> forward;
		Heuristic<MNPuzzleState> reverse;

		MNPuzzle puzzle(sz, sz);
		start.Reset();
		goal.Reset();

		std::vector<MNPuzzleState> puzzle_vector;
		MNPuzzle::Create_Random_MN_Puzzles(goal, puzzle_vector, 20);
		//std::vector<slideDir> actions;
		//srand(time(0));
		//for (int i = 0; i < 10; i++)
		//{
		//	puzzle.GetActions(start, actions);
		//	puzzle.ApplyAction(start, actions[rand()%actions.size()]);
		//}


		start = puzzle_vector[atoi(argv[2])];

		BuildHeuristics(start, goal, forward,sz);
		BuildHeuristics(goal, start, reverse,sz);


		
		PEMMMNPuzzle *searcher;

		std::cout << "Start: " << start << std::endl;
		std::cout << "Goal: " << goal << std::endl;
		searcher = new PEMMMNPuzzle(start, goal, argv[3], argv[4], forward, reverse, &puzzle);
		searcher->FindAPath();
	}
	else if (strcmp(argv[1], "-mnpuzzle4") == 0)
	{
		hprefix = argv[5];
		int sz = 4;
		MNPuzzleState start(sz, sz);
		MNPuzzleState goal(sz, sz);

		Heuristic<MNPuzzleState> forward;
		Heuristic<MNPuzzleState> reverse;

		MNPuzzle puzzle(sz, sz);
		start.Reset();
		goal.Reset();

		GetMNPuzzleInstance(atoi(argv[2]), start);
		//std::vector<slideDir> actions;
		//srand(time(0));
		//for (int i = 0; i < 10; i++)
		//{
		//	puzzle.GetActions(start, actions);
		//	puzzle.ApplyAction(start, actions[rand()%actions.size()]);
		//}

		BuildHeuristics(start, goal, forward, sz);
		BuildHeuristics(goal, start, reverse, sz);



		PEMMMNPuzzle *searcher;

		std::cout << "Start: " << start << std::endl;
		std::cout << "Goal: " << goal << std::endl;
		searcher = new PEMMMNPuzzle(start, goal, argv[3], argv[4], forward, reverse, &puzzle);
		searcher->FindAPath();
	}
	else {
		InstallHandlers();
		RunHOGGUI(argc, argv);
	}
}


void BuildHeuristics(RubiksState start, RubiksState goal, Heuristic<RubiksState> &result)
{
	RubiksCube cube;
	std::vector<int> blank;
#ifdef ZERO
	ZeroHeuristic<RubiksState> *zero = new ZeroHeuristic<RubiksState>();
	result.lookups.push_back({ kLeafNode, 0, 0 });
	result.heuristics.push_back(zero);
#endif

#ifdef TINY
	std::vector<int> edges1 = { 1, 3, 8, 9 }; // first 4
	std::vector<int> edges2 = { 0, 2, 4, 5 }; // first 4
	std::vector<int> corners = { 0, 1, 2, 3 }; // first 4
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
	if (!pdb1->Load(hprefix))
	{
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1->Save(hprefix);
	}
	if (!pdb2->Load(hprefix))
	{
		pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2->Save(hprefix);
	}
	if (!pdb3->Load(hprefix))
	{
		pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3->Save(hprefix);
	}
	result.lookups.push_back({ kMaxNode, 1, 3 });
	result.lookups.push_back({ kLeafNode, 0, 0 });
	result.lookups.push_back({ kLeafNode, 1, 0 });
	result.lookups.push_back({ kLeafNode, 2, 0 });
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
#endif


#ifdef SMALL
	std::vector<int> edges1 = { 0, 1, 2, 4, 6 };
	std::vector<int> edges2 = { 3, 5 };
	std::vector<int> edges3 = { 7, 8, 9, 10, 11 };
	std::vector<int> corners1 = { 0, 1, 2, 3, 4, 5 };
	std::vector<int> corners2 = { 2, 3, 4, 5, 6, 7 };
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, edges3, blank);
	RubikPDB *pdb4 = new RubikPDB(&cube, goal, blank, corners1);
	RubikPDB *pdb5 = new RubikPDB(&cube, goal, blank, corners2);
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb5->BuildPDB(goal, std::thread::hardware_concurrency());
	result.lookups.push_back({ kMaxNode, 1, 5 });
	result.lookups.push_back({ kLeafNode, 0, 0 });
	result.lookups.push_back({ kLeafNode, 1, 0 });
	result.lookups.push_back({ kLeafNode, 2, 0 });
	result.lookups.push_back({ kLeafNode, 3, 0 });
	result.lookups.push_back({ kLeafNode, 4, 0 });
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
	result.heuristics.push_back(pdb4);
	result.heuristics.push_back(pdb5);
#endif

#ifdef KORF97
	std::vector<int> edges1 = { 1, 3, 8, 9, 10, 11 };
	std::vector<int> edges2 = { 0, 2, 4, 5, 6, 7 };
	std::vector<int> corners = { 0, 1, 2, 3, 4, 5, 6, 7 };
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);

	//	assert(!"File names are getting corrupted here. Perhaps by the abstraction of the goal state");
	//	assert(!"Need to abstract the goal state immediately when creating the pdb instead of only when I build the pdb");
	if (!pdb1->Load(hprefix))
	{
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	if (!pdb2->Load(hprefix))
	{
		pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	if (!pdb3->Load(hprefix))
	{
		pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3->Save(hprefix);
	}
	else {
		printf("Loaded previous heuristic\n");
	}
	result.lookups.push_back({ kMaxNode, 1, 3 });
	result.lookups.push_back({ kLeafNode, 0, 0 });
	result.lookups.push_back({ kLeafNode, 1, 0 });
	result.lookups.push_back({ kLeafNode, 2, 0 });
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
#endif

#ifdef MASSIVE
	std::vector<int> edges1 = { 0, 1, 2, 3, 4, 5, 6, 7 };
	std::vector<int> edges2 = { 1, 3, 5, 7, 8, 9, 10, 11 };
	std::vector<int> corners = { 0, 1, 2, 3, 4, 5, 6, 7 }; // first 4
	RubikPDB *pdb1 = new RubikPDB(&cube, goal, edges1, blank);
	RubikPDB *pdb2 = new RubikPDB(&cube, goal, edges2, blank);
	RubikPDB *pdb3 = new RubikPDB(&cube, goal, blank, corners);
	if (!pdb1->Load(hprefix))
	{
		pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1->Save(hprefix);
	}
	if (!pdb2->Load(hprefix))
	{
		pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2->Save(hprefix);
	}
	if (!pdb3->Load(hprefix))
	{
		pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3->Save(hprefix);
	}
	result.lookups.push_back({ kMaxNode, 1, 3 });
	result.lookups.push_back({ kLeafNode, 0, 0 });
	result.lookups.push_back({ kLeafNode, 1, 0 });
	result.lookups.push_back({ kLeafNode, 2, 0 });
	result.heuristics.push_back(pdb1);
	result.heuristics.push_back(pdb2);
	result.heuristics.push_back(pdb3);
#endif

}


void BuildHeuristics(int count, PancakePuzzleState start, PancakePuzzleState goal, Heuristic<PancakePuzzleState> &result)
{
	PancakePuzzle pancake(start.puzzle.size());
	std::vector<int> pdb;
	for (int x = 0; x < count; x++)
		pdb.push_back(x);

	
	//PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> *pdb1;
	//pdb1 = new PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle>(&pancake, goal, pdb);
	////pdb1->BuildPDB(goal, 0, std::thread::hardware_concurrency());
	PancakePDB* pdb1;
	pdb1 = new PancakePDB(&pancake, goal, pdb);
	pdb1->BuildPDB(goal,std::thread::hardware_concurrency());
	//if (!pdb1->Load(hprefix))
	//{
	//	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	//	pdb1->Save(hprefix);
	//}
	result.lookups.push_back({ kLeafNode, 0, 0 });
	result.heuristics.push_back(pdb1);
	//pdb3->Save(hprefix);
}

void BuildHeuristics(MNPuzzleState start, MNPuzzleState goal, Heuristic<MNPuzzleState>& result,int size)
{
	std::vector<int> pattern = { 0, 1, 2, 3, 4, 5, 6 };
	MNPuzzle puzzle(size, size);
	puzzle.StoreGoal(goal);
	MNPuzzlePDB* pdb = new MNPuzzlePDB(&puzzle, goal, pattern);
	//MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
	if (!pdb->Load(hprefix))
	{
		pdb->BuildPDB(goal, std::thread::hardware_concurrency());
		pdb->Save(hprefix);
	}
	//pdb->BuildPDB(goal, std::thread::hardware_concurrency());
	result.lookups.push_back({ kLeafNode, 0, 0 });
	std::cout << "result.lookups.size(): "<<result.lookups.size()<<"\n";
	result.heuristics.push_back(pdb);
}

void GetInstance(int which, PancakePuzzleState &s)
{
	int states[309][10] =
	{ { 0, 1, 2, 3, 4, 5, 6, 7, 9, 8 },
	{ 0, 1, 2, 3, 4, 5, 6, 9, 8, 7 },
	{ 0, 1, 2, 3, 4, 5, 9, 8, 7, 6 },
	{ 0, 1, 2, 3, 4, 9, 8, 7, 6, 5 },
	{ 0, 1, 2, 3, 9, 8, 7, 6, 5, 4 },
	{ 0, 1, 2, 9, 8, 7, 6, 5, 4, 3 },
	{ 0, 1, 9, 8, 7, 6, 5, 4, 3, 2 },
	{ 0, 9, 8, 7, 6, 5, 4, 3, 2, 1 },
	{ 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 8, 9, 7 },
	{ 0, 1, 2, 3, 4, 5, 8, 9, 7, 6 },
	{ 0, 1, 2, 3, 4, 8, 9, 7, 6, 5 },
	{ 0, 1, 2, 3, 8, 9, 7, 6, 5, 4 },
	{ 0, 1, 2, 8, 9, 7, 6, 5, 4, 3 },
	{ 0, 1, 8, 9, 7, 6, 5, 4, 3, 2 },
	{ 0, 8, 9, 7, 6, 5, 4, 3, 2, 1 },
	{ 8, 9, 7, 6, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 5, 6, 9, 7, 8 },
	{ 0, 1, 2, 3, 4, 5, 7, 8, 9, 6 },
	{ 0, 1, 2, 3, 4, 7, 8, 9, 6, 5 },
	{ 0, 1, 2, 3, 7, 8, 9, 6, 5, 4 },
	{ 0, 1, 2, 7, 8, 9, 6, 5, 4, 3 },
	{ 0, 1, 7, 8, 9, 6, 5, 4, 3, 2 },
	{ 0, 7, 8, 9, 6, 5, 4, 3, 2, 1 },
	{ 7, 8, 9, 6, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 5, 9, 8, 6, 7 },
	{ 0, 1, 2, 3, 4, 5, 9, 6, 7, 8 },
	{ 0, 1, 2, 3, 4, 6, 7, 8, 9, 5 },
	{ 0, 1, 2, 3, 6, 7, 8, 9, 5, 4 },
	{ 0, 1, 2, 6, 7, 8, 9, 5, 4, 3 },
	{ 0, 1, 6, 7, 8, 9, 5, 4, 3, 2 },
	{ 0, 6, 7, 8, 9, 5, 4, 3, 2, 1 },
	{ 6, 7, 8, 9, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 9, 8, 7, 5, 6 },
	{ 0, 1, 2, 3, 4, 9, 8, 5, 6, 7 },
	{ 0, 1, 2, 3, 4, 9, 5, 6, 7, 8 },
	{ 0, 1, 2, 3, 5, 6, 7, 8, 9, 4 },
	{ 0, 1, 2, 5, 6, 7, 8, 9, 4, 3 },
	{ 0, 1, 5, 6, 7, 8, 9, 4, 3, 2 },
	{ 0, 1, 2, 3, 4, 5, 6, 8, 7, 9 },
	{ 0, 1, 2, 3, 4, 5, 7, 9, 8, 6 },
	{ 0, 1, 2, 3, 4, 7, 9, 8, 6, 5 },
	{ 0, 1, 2, 3, 7, 9, 8, 6, 5, 4 },
	{ 0, 1, 2, 7, 9, 8, 6, 5, 4, 3 },
	{ 0, 1, 7, 9, 8, 6, 5, 4, 3, 2 },
	{ 0, 7, 9, 8, 6, 5, 4, 3, 2, 1 },
	{ 7, 9, 8, 6, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 5, 8, 9, 6, 7 },
	{ 0, 1, 2, 3, 4, 5, 8, 6, 7, 9 },
	{ 0, 1, 2, 3, 4, 6, 7, 9, 8, 5 },
	{ 0, 1, 2, 3, 6, 7, 9, 8, 5, 4 },
	{ 0, 1, 2, 6, 7, 9, 8, 5, 4, 3 },
	{ 0, 1, 6, 7, 9, 8, 5, 4, 3, 2 },
	{ 0, 6, 7, 9, 8, 5, 4, 3, 2, 1 },
	{ 6, 7, 9, 8, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 8, 9, 7, 5, 6 },
	{ 0, 1, 2, 3, 4, 8, 9, 5, 6, 7 },
	{ 0, 1, 2, 3, 4, 8, 5, 6, 7, 9 },
	{ 0, 1, 2, 3, 5, 6, 7, 9, 8, 4 },
	{ 0, 1, 2, 5, 6, 7, 9, 8, 4, 3 },
	{ 0, 1, 5, 6, 7, 9, 8, 4, 3, 2 },
	{ 0, 5, 6, 7, 9, 8, 4, 3, 2, 1 },
	{ 5, 6, 7, 9, 8, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 8, 9, 7, 6, 4, 5 },
	{ 0, 1, 2, 3, 8, 9, 7, 4, 5, 6 },
	{ 0, 1, 2, 3, 8, 9, 4, 5, 6, 7 },
	{ 0, 1, 2, 3, 8, 4, 5, 6, 7, 9 },
	{ 0, 1, 2, 4, 5, 6, 7, 9, 8, 3 },
	{ 0, 1, 4, 5, 6, 7, 9, 8, 3, 2 },
	{ 0, 1, 2, 3, 4, 5, 9, 7, 8, 6 },
	{ 0, 1, 2, 3, 4, 9, 7, 8, 6, 5 },
	{ 0, 1, 2, 3, 9, 7, 8, 6, 5, 4 },
	{ 0, 1, 2, 9, 7, 8, 6, 5, 4, 3 },
	{ 0, 1, 9, 7, 8, 6, 5, 4, 3, 2 },
	{ 0, 9, 7, 8, 6, 5, 4, 3, 2, 1 },
	{ 9, 7, 8, 6, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 5, 7, 9, 6, 8 },
	{ 0, 1, 2, 3, 4, 6, 8, 9, 7, 5 },
	{ 0, 1, 2, 3, 6, 8, 9, 7, 5, 4 },
	{ 0, 1, 2, 6, 8, 9, 7, 5, 4, 3 },
	{ 0, 1, 6, 8, 9, 7, 5, 4, 3, 2 },
	{ 0, 6, 8, 9, 7, 5, 4, 3, 2, 1 },
	{ 6, 8, 9, 7, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 7, 9, 8, 5, 6 },
	{ 0, 1, 2, 3, 4, 7, 9, 5, 6, 8 },
	{ 0, 1, 2, 3, 4, 7, 5, 6, 8, 9 },
	{ 0, 1, 2, 3, 5, 6, 8, 9, 7, 4 },
	{ 0, 1, 2, 5, 6, 8, 9, 7, 4, 3 },
	{ 0, 1, 5, 6, 8, 9, 7, 4, 3, 2 },
	{ 0, 5, 6, 8, 9, 7, 4, 3, 2, 1 },
	{ 5, 6, 8, 9, 7, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 7, 9, 8, 6, 4, 5 },
	{ 0, 1, 2, 3, 7, 9, 8, 4, 5, 6 },
	{ 0, 1, 2, 3, 7, 9, 4, 5, 6, 8 },
	{ 0, 1, 2, 3, 7, 4, 5, 6, 8, 9 },
	{ 0, 1, 2, 4, 5, 6, 8, 9, 7, 3 },
	{ 0, 1, 4, 5, 6, 8, 9, 7, 3, 2 },
	{ 0, 4, 5, 6, 8, 9, 7, 3, 2, 1 },
	{ 4, 5, 6, 8, 9, 7, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 4, 6, 8, 7, 9, 5 },
	{ 0, 1, 2, 3, 6, 8, 7, 9, 5, 4 },
	{ 0, 1, 2, 6, 8, 7, 9, 5, 4, 3 },
	{ 0, 1, 6, 8, 7, 9, 5, 4, 3, 2 },
	{ 0, 6, 8, 7, 9, 5, 4, 3, 2, 1 },
	{ 6, 8, 7, 9, 5, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 5, 6, 8, 7, 9, 4 },
	{ 0, 1, 2, 5, 6, 8, 7, 9, 4, 3 },
	{ 0, 1, 5, 6, 8, 7, 9, 4, 3, 2 },
	{ 0, 5, 6, 8, 7, 9, 4, 3, 2, 1 },
	{ 5, 6, 8, 7, 9, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 9, 7, 8, 6, 4, 5 },
	{ 0, 1, 2, 4, 5, 6, 8, 7, 9, 3 },
	{ 0, 1, 4, 5, 6, 8, 7, 9, 3, 2 },
	{ 0, 4, 5, 6, 8, 7, 9, 3, 2, 1 },
	{ 4, 5, 6, 8, 7, 9, 3, 2, 1, 0 },
	{ 0, 1, 2, 9, 7, 8, 6, 5, 3, 4 },
	{ 0, 1, 2, 9, 7, 8, 6, 3, 4, 5 },
	{ 0, 1, 3, 4, 5, 6, 8, 7, 9, 2 },
	{ 0, 3, 4, 5, 6, 8, 7, 9, 2, 1 },
	{ 3, 4, 5, 6, 8, 7, 9, 2, 1, 0 },
	{ 0, 1, 9, 7, 8, 6, 5, 4, 2, 3 },
	{ 0, 1, 9, 7, 8, 6, 5, 2, 3, 4 },
	{ 0, 1, 9, 7, 8, 6, 2, 3, 4, 5 },
	{ 0, 2, 3, 4, 5, 6, 8, 7, 9, 1 },
	{ 2, 3, 4, 5, 6, 8, 7, 9, 1, 0 },
	{ 0, 9, 7, 8, 6, 5, 4, 3, 1, 2 },
	{ 0, 9, 7, 8, 6, 5, 4, 1, 2, 3 },
	{ 0, 9, 7, 8, 6, 5, 1, 2, 3, 4 },
	{ 0, 9, 7, 8, 6, 1, 2, 3, 4, 5 },
	{ 0, 1, 2, 3, 5, 9, 7, 8, 6, 4 },
	{ 0, 1, 2, 5, 9, 7, 8, 6, 4, 3 },
	{ 0, 1, 5, 9, 7, 8, 6, 4, 3, 2 },
	{ 0, 5, 9, 7, 8, 6, 4, 3, 2, 1 },
	{ 5, 9, 7, 8, 6, 4, 3, 2, 1, 0 },
	{ 0, 1, 2, 3, 6, 8, 7, 9, 4, 5 },
	{ 0, 1, 2, 4, 5, 9, 7, 8, 6, 3 },
	{ 0, 1, 4, 5, 9, 7, 8, 6, 3, 2 },
	{ 0, 4, 5, 9, 7, 8, 6, 3, 2, 1 },
	{ 4, 5, 9, 7, 8, 6, 3, 2, 1, 0 },
	{ 0, 1, 2, 6, 8, 7, 9, 5, 3, 4 },
	{ 0, 1, 2, 6, 8, 7, 9, 3, 4, 5 },
	{ 0, 1, 3, 4, 5, 9, 7, 8, 6, 2 },
	{ 0, 3, 4, 5, 9, 7, 8, 6, 2, 1 },
	{ 3, 4, 5, 9, 7, 8, 6, 2, 1, 0 },
	{ 0, 1, 6, 8, 7, 9, 5, 4, 2, 3 },
	{ 0, 1, 6, 8, 7, 9, 5, 2, 3, 4 },
	{ 0, 1, 6, 8, 7, 9, 2, 3, 4, 5 },
	{ 0, 2, 3, 4, 5, 9, 7, 8, 6, 1 },
	{ 2, 3, 4, 5, 9, 7, 8, 6, 1, 0 },
	{ 0, 6, 8, 7, 9, 5, 4, 3, 1, 2 },
	{ 0, 6, 8, 7, 9, 5, 4, 1, 2, 3 },
	{ 0, 6, 8, 7, 9, 5, 1, 2, 3, 4 },
	{ 0, 6, 8, 7, 9, 1, 2, 3, 4, 5 },
	{ 1, 2, 3, 4, 5, 9, 7, 8, 6, 0 },
	{ 6, 8, 7, 9, 5, 4, 3, 2, 0, 1 },
	{ 6, 8, 7, 9, 5, 4, 3, 0, 1, 2 },
	{ 6, 8, 7, 9, 5, 4, 0, 1, 2, 3 },
	{ 6, 8, 7, 9, 5, 0, 1, 2, 3, 4 },
	{ 6, 8, 7, 9, 0, 1, 2, 3, 4, 5 },
	{ 0, 1, 2, 4, 6, 8, 7, 9, 5, 3 },
	{ 0, 1, 4, 6, 8, 7, 9, 5, 3, 2 },
	{ 0, 4, 6, 8, 7, 9, 5, 3, 2, 1 },
	{ 4, 6, 8, 7, 9, 5, 3, 2, 1, 0 },
	{ 0, 1, 2, 5, 9, 7, 8, 6, 3, 4 },
	{ 0, 1, 2, 5, 3, 4, 6, 8, 7, 9 },
	{ 0, 1, 3, 4, 6, 8, 7, 9, 5, 2 },
	{ 0, 3, 4, 6, 8, 7, 9, 5, 2, 1 },
	{ 3, 4, 6, 8, 7, 9, 5, 2, 1, 0 },
	{ 0, 1, 5, 9, 7, 8, 6, 4, 2, 3 },
	{ 0, 1, 5, 9, 7, 8, 6, 2, 3, 4 },
	{ 0, 1, 5, 2, 3, 4, 6, 8, 7, 9 },
	{ 0, 2, 3, 4, 6, 8, 7, 9, 5, 1 },
	{ 2, 3, 4, 6, 8, 7, 9, 5, 1, 0 },
	{ 0, 5, 9, 7, 8, 6, 4, 3, 1, 2 },
	{ 0, 5, 9, 7, 8, 6, 4, 1, 2, 3 },
	{ 0, 5, 9, 7, 8, 6, 1, 2, 3, 4 },
	{ 0, 5, 1, 2, 3, 4, 6, 8, 7, 9 },
	{ 1, 2, 3, 4, 6, 8, 7, 9, 5, 0 },
	{ 5, 9, 7, 8, 6, 4, 3, 2, 0, 1 },
	{ 5, 9, 7, 8, 6, 4, 3, 0, 1, 2 },
	{ 5, 9, 7, 8, 6, 4, 0, 1, 2, 3 },
	{ 5, 9, 7, 8, 6, 0, 1, 2, 3, 4 },
	{ 5, 0, 1, 2, 3, 4, 6, 8, 7, 9 },
	{ 0, 1, 2, 5, 4, 9, 7, 8, 6, 3 },
	{ 0, 1, 5, 4, 9, 7, 8, 6, 3, 2 },
	{ 0, 5, 4, 9, 7, 8, 6, 3, 2, 1 },
	{ 5, 4, 9, 7, 8, 6, 3, 2, 1, 0 },
	{ 0, 1, 2, 4, 5, 3, 6, 8, 7, 9 },
	{ 0, 1, 2, 4, 3, 6, 8, 7, 9, 5 },
	{ 0, 1, 3, 5, 9, 7, 8, 6, 4, 2 },
	{ 0, 3, 5, 9, 7, 8, 6, 4, 2, 1 },
	{ 3, 5, 9, 7, 8, 6, 4, 2, 1, 0 },
	{ 0, 1, 4, 6, 8, 7, 9, 5, 2, 3 },
	{ 0, 1, 4, 6, 8, 7, 9, 2, 3, 5 },
	{ 0, 1, 4, 2, 3, 5, 9, 7, 8, 6 },
	{ 0, 2, 3, 5, 9, 7, 8, 6, 4, 1 },
	{ 2, 3, 5, 9, 7, 8, 6, 4, 1, 0 },
	{ 0, 4, 6, 8, 7, 9, 5, 3, 1, 2 },
	{ 0, 4, 6, 8, 7, 9, 5, 1, 2, 3 },
	{ 0, 4, 6, 8, 7, 9, 1, 2, 3, 5 },
	{ 0, 4, 1, 2, 3, 5, 9, 7, 8, 6 },
	{ 1, 2, 3, 5, 9, 7, 8, 6, 4, 0 },
	{ 4, 6, 8, 7, 9, 5, 3, 2, 0, 1 },
	{ 4, 6, 8, 7, 9, 5, 3, 0, 1, 2 },
	{ 4, 6, 8, 7, 9, 5, 0, 1, 2, 3 },
	{ 4, 6, 8, 7, 9, 0, 1, 2, 3, 5 },
	{ 4, 0, 1, 2, 3, 5, 9, 7, 8, 6 },
	{ 0, 1, 4, 3, 6, 8, 7, 9, 5, 2 },
	{ 0, 4, 3, 6, 8, 7, 9, 5, 2, 1 },
	{ 4, 3, 6, 8, 7, 9, 5, 2, 1, 0 },
	{ 0, 9, 7, 8, 6, 4, 3, 5, 2, 1 },
	{ 9, 7, 8, 6, 4, 3, 5, 2, 1, 0 },
	{ 0, 1, 3, 4, 2, 5, 9, 7, 8, 6 },
	{ 0, 1, 3, 2, 5, 9, 7, 8, 6, 4 },
	{ 0, 2, 5, 9, 7, 8, 6, 4, 3, 1 },
	{ 2, 5, 9, 7, 8, 6, 4, 3, 1, 0 },
	{ 0, 3, 4, 6, 8, 7, 9, 5, 1, 2 },
	{ 0, 3, 4, 6, 8, 7, 9, 1, 2, 5 },
	{ 0, 3, 4, 1, 2, 5, 9, 7, 8, 6 },
	{ 0, 1, 3, 5, 2, 4, 6, 8, 7, 9 },
	{ 0, 2, 4, 6, 8, 7, 9, 5, 3, 1 },
	{ 2, 4, 6, 8, 7, 9, 5, 3, 1, 0 },
	{ 0, 3, 5, 9, 7, 8, 6, 4, 1, 2 },
	{ 0, 3, 5, 9, 7, 8, 6, 1, 2, 4 },
	{ 0, 3, 5, 1, 2, 4, 6, 8, 7, 9 },
	{ 0, 3, 1, 2, 4, 6, 8, 7, 9, 5 },
	{ 1, 2, 4, 6, 8, 7, 9, 5, 3, 0 },
	{ 3, 5, 9, 7, 8, 6, 4, 2, 0, 1 },
	{ 3, 5, 9, 7, 8, 6, 4, 0, 1, 2 },
	{ 3, 5, 9, 7, 8, 6, 0, 1, 2, 4 },
	{ 3, 5, 0, 1, 2, 4, 6, 8, 7, 9 },
	{ 3, 0, 1, 2, 4, 6, 8, 7, 9, 5 },
	{ 0, 3, 2, 5, 9, 7, 8, 6, 4, 1 },
	{ 3, 2, 5, 9, 7, 8, 6, 4, 1, 0 },
	{ 0, 5, 3, 2, 9, 7, 8, 6, 4, 1 },
	{ 5, 3, 2, 9, 7, 8, 6, 4, 1, 0 },
	{ 6, 8, 7, 9, 5, 3, 2, 4, 1, 0 },
	{ 0, 2, 3, 5, 1, 4, 6, 8, 7, 9 },
	{ 0, 2, 3, 1, 4, 6, 8, 7, 9, 5 },
	{ 0, 2, 1, 4, 6, 8, 7, 9, 5, 3 },
	{ 1, 4, 6, 8, 7, 9, 5, 3, 2, 0 },
	{ 2, 3, 5, 9, 7, 8, 6, 4, 0, 1 },
	{ 2, 3, 5, 9, 7, 8, 6, 0, 1, 4 },
	{ 2, 3, 5, 0, 1, 4, 6, 8, 7, 9 },
	{ 2, 3, 0, 1, 4, 6, 8, 7, 9, 5 },
	{ 2, 0, 1, 4, 6, 8, 7, 9, 5, 3 },
	{ 0, 4, 6, 8, 7, 9, 5, 2, 1, 3 },
	{ 0, 4, 2, 1, 3, 5, 9, 7, 8, 6 },
	{ 0, 2, 1, 3, 5, 9, 7, 8, 6, 4 },
	{ 9, 7, 8, 6, 4, 2, 5, 3, 1, 0 },
	{ 0, 2, 4, 1, 3, 5, 9, 7, 8, 6 },
	{ 1, 3, 5, 9, 7, 8, 6, 4, 2, 0 },
	{ 2, 4, 6, 8, 7, 9, 5, 3, 0, 1 },
	{ 2, 4, 6, 8, 7, 9, 5, 0, 1, 3 },
	{ 2, 4, 6, 8, 7, 9, 0, 1, 3, 5 },
	{ 2, 4, 0, 1, 3, 5, 9, 7, 8, 6 },
	{ 2, 0, 1, 3, 5, 9, 7, 8, 6, 4 },
	{ 2, 1, 4, 6, 8, 7, 9, 5, 3, 0 },
	{ 4, 2, 1, 6, 8, 7, 9, 5, 3, 0 },
	{ 1, 2, 4, 0, 3, 5, 9, 7, 8, 6 },
	{ 1, 2, 0, 3, 5, 9, 7, 8, 6, 4 },
	{ 1, 0, 3, 5, 9, 7, 8, 6, 4, 2 },
	{ 3, 5, 9, 7, 8, 6, 4, 1, 0, 2 },
	{ 3, 5, 9, 7, 8, 6, 1, 0, 2, 4 },
	{ 3, 5, 1, 0, 2, 4, 6, 8, 7, 9 },
	{ 3, 1, 0, 2, 4, 6, 8, 7, 9, 5 },
	{ 1, 0, 2, 4, 6, 8, 7, 9, 5, 3 },
	{ 3, 5, 9, 7, 8, 6, 4, 0, 2, 1 },
	{ 3, 5, 9, 7, 8, 6, 0, 1, 4, 2 },
	{ 3, 5, 9, 7, 8, 6, 0, 4, 2, 1 },
	{ 3, 5, 0, 9, 7, 8, 6, 4, 2, 1 },
	{ 3, 0, 5, 9, 7, 8, 6, 4, 2, 1 },
	{ 1, 4, 6, 8, 7, 9, 5, 2, 3, 0 },
	{ 3, 2, 5, 9, 7, 8, 6, 0, 1, 4 },
	{ 1, 4, 6, 8, 7, 9, 2, 3, 5, 0 },
	{ 3, 5, 9, 7, 8, 6, 4, 1, 2, 0 },
	{ 1, 4, 6, 8, 7, 9, 5, 3, 0, 2 },
	{ 1, 4, 6, 8, 7, 9, 5, 0, 2, 3 },
	{ 1, 4, 6, 8, 7, 9, 0, 2, 3, 5 },
	{ 1, 3, 5, 0, 2, 4, 6, 8, 7, 9 },
	{ 1, 3, 0, 2, 4, 6, 8, 7, 9, 5 },
	{ 3, 1, 2, 0, 4, 6, 8, 7, 9, 5 },
	{ 3, 5, 9, 7, 8, 6, 0, 2, 4, 1 },
	{ 1, 4, 6, 8, 7, 9, 5, 2, 0, 3 },
	{ 1, 4, 2, 0, 3, 5, 9, 7, 8, 6 },
	{ 2, 0, 3, 5, 1, 4, 6, 8, 7, 9 },
	{ 2, 0, 3, 1, 4, 6, 8, 7, 9, 5 },
	{ 4, 0, 2, 1, 3, 5, 9, 7, 8, 6 },
	{ 2, 0, 4, 6, 8, 7, 9, 5, 1, 3 },
	{ 2, 0, 4, 1, 3, 5, 9, 7, 8, 6 },
	{ 3, 1, 5, 9, 7, 8, 6, 4, 0, 2 },
	{ 2, 5, 3, 1, 9, 7, 8, 6, 4, 0 },
	{ 5, 3, 1, 9, 7, 8, 6, 4, 0, 2 },
	{ 1, 3, 5, 2, 0, 4, 6, 8, 7, 9 },
	{ 4, 1, 6, 8, 7, 9, 5, 0, 3, 2 },
	{ 2, 5, 9, 7, 8, 6, 3, 0, 4, 1 },
	{ 1, 4, 0, 3, 6, 8, 7, 9, 5, 2 },
	{ 3, 1, 4, 0, 2, 5, 9, 7, 8, 6 },
	{ 2, 5, 1, 3, 0, 4, 6, 8, 7, 9 },
	{ 4, 1, 2, 5, 0, 3, 6, 8, 7, 9 },
	{ 2, 5, 9, 7, 8, 6, 0, 3, 1, 4 },
	{ 2, 5, 0, 3, 1, 4, 6, 8, 7, 9 },
	{ 4, 1, 3, 0, 2, 5, 9, 7, 8, 6 },
	{ 4, 6, 8, 7, 9, 0, 3, 1, 5, 2 },
	{ 1, 3, 0, 4, 2, 5, 9, 7, 8, 6 },
	{ 3, 1, 5, 2, 0, 9, 7, 8, 6, 4 },
	{ 3, 1, 6, 8, 7, 9, 5, 2, 0, 4 },
	{ 3, 0, 2, 5, 1, 4, 6, 8, 7, 9 },
	{ 1, 3, 2, 4, 0, 5, 9, 7, 8, 6 } };
	for (int x = 0; x < 10; x++)
	{
		s.puzzle[x] = 9 - states[which][9 - x];
	}
}

void GetMNPuzzleInstance(int which, MNPuzzleState &s)
{
	int states[110][16] =
	{
		//easy ones
		{ 4, 1, 2, 3, 9, 8, 6, 7, 12, 5, 10, 11, 13, 0, 14, 15 },
		{ 4, 1, 2, 3, 5, 6, 10, 7, 8, 9, 14, 11, 12, 0, 13, 15 },
		{ 4, 1, 2, 3, 8, 0, 6, 7, 9, 5, 10, 11, 12, 13, 14, 15 },
		{ 1, 2, 6, 3, 4, 5, 10, 7, 8, 9, 11, 15, 12, 13, 14, 0 },
		{ 4, 1, 2, 3, 8, 0, 6, 7, 9, 5, 10, 11, 12, 13, 14, 15 },
		{ 5, 4, 2, 3, 1, 0, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
		{ 1, 2, 3, 7, 4, 5, 6, 11, 8, 9, 14, 10, 12, 0, 13, 15 },
		{ 4, 1, 2, 3, 5, 6, 7, 0, 8, 9, 10, 11, 12, 13, 14, 15 },
		{ 4, 1, 2, 3, 5, 6, 7, 0, 8, 9, 10, 11, 12, 13, 14, 15 },
		{ 1, 5, 2, 3, 4, 9, 6, 7, 0, 13, 10, 11, 8, 12, 14, 15 },
		//std test cases
		{ 14, 13, 15, 7, 11, 12, 9, 5, 6, 0, 2, 1, 4, 8, 10, 3 },
		{ 13, 5, 4, 10, 9, 12, 8, 14, 2, 3, 7, 1, 0, 15, 11, 6 },
		{ 14, 7, 8, 2, 13, 11, 10, 4, 9, 12, 5, 0, 3, 6, 1, 15 },
		{ 5, 12, 10, 7, 15, 11, 14, 0, 8, 2, 1, 13, 3, 4, 9, 6 },
		{ 4, 7, 14, 13, 10, 3, 9, 12, 11, 5, 6, 15, 1, 2, 8, 0 },
		{ 14, 7, 1, 9, 12, 3, 6, 15, 8, 11, 2, 5, 10, 0, 4, 13 },
		{ 2, 11, 15, 5, 13, 4, 6, 7, 12, 8, 10, 1, 9, 3, 14, 0 },
		{ 12, 11, 15, 3, 8, 0, 4, 2, 6, 13, 9, 5, 14, 1, 10, 7 },
		{ 3, 14, 9, 11, 5, 4, 8, 2, 13, 12, 6, 7, 10, 1, 15, 0 },
		{ 13, 11, 8, 9, 0, 15, 7, 10, 4, 3, 6, 14, 5, 12, 2, 1 },
		{ 5, 9, 13, 14, 6, 3, 7, 12, 10, 8, 4, 0, 15, 2, 11, 1 },
		{ 14, 1, 9, 6, 4, 8, 12, 5, 7, 2, 3, 0, 10, 11, 13, 15 },
		{ 3, 6, 5, 2, 10, 0, 15, 14, 1, 4, 13, 12, 9, 8, 11, 7 },
		{ 7, 6, 8, 1, 11, 5, 14, 10, 3, 4, 9, 13, 15, 2, 0, 12 },
		{ 13, 11, 4, 12, 1, 8, 9, 15, 6, 5, 14, 2, 7, 3, 10, 0 },
		{ 1, 3, 2, 5, 10, 9, 15, 6, 8, 14, 13, 11, 12, 4, 7, 0 },
		{ 15, 14, 0, 4, 11, 1, 6, 13, 7, 5, 8, 9, 3, 2, 10, 12 },
		{ 6, 0, 14, 12, 1, 15, 9, 10, 11, 4, 7, 2, 8, 3, 5, 13 },
		{ 7, 11, 8, 3, 14, 0, 6, 15, 1, 4, 13, 9, 5, 12, 2, 10 },
		{ 6, 12, 11, 3, 13, 7, 9, 15, 2, 14, 8, 10, 4, 1, 5, 0 },
		{ 12, 8, 14, 6, 11, 4, 7, 0, 5, 1, 10, 15, 3, 13, 9, 2 },
		{ 14, 3, 9, 1, 15, 8, 4, 5, 11, 7, 10, 13, 0, 2, 12, 6 },
		{ 10, 9, 3, 11, 0, 13, 2, 14, 5, 6, 4, 7, 8, 15, 1, 12 },
		{ 7, 3, 14, 13, 4, 1, 10, 8, 5, 12, 9, 11, 2, 15, 6, 0 },
		{ 11, 4, 2, 7, 1, 0, 10, 15, 6, 9, 14, 8, 3, 13, 5, 12 },
		{ 5, 7, 3, 12, 15, 13, 14, 8, 0, 10, 9, 6, 1, 4, 2, 11 },
		{ 14, 1, 8, 15, 2, 6, 0, 3, 9, 12, 10, 13, 4, 7, 5, 11 },
		{ 13, 14, 6, 12, 4, 5, 1, 0, 9, 3, 10, 2, 15, 11, 8, 7 },
		{ 9, 8, 0, 2, 15, 1, 4, 14, 3, 10, 7, 5, 11, 13, 6, 12 },
		{ 12, 15, 2, 6, 1, 14, 4, 8, 5, 3, 7, 0, 10, 13, 9, 11 },
		{ 12, 8, 15, 13, 1, 0, 5, 4, 6, 3, 2, 11, 9, 7, 14, 10 },
		{ 14, 10, 9, 4, 13, 6, 5, 8, 2, 12, 7, 0, 1, 3, 11, 15 },
		{ 14, 3, 5, 15, 11, 6, 13, 9, 0, 10, 2, 12, 4, 1, 7, 8 },
		{ 6, 11, 7, 8, 13, 2, 5, 4, 1, 10, 3, 9, 14, 0, 12, 15 },
		{ 1, 6, 12, 14, 3, 2, 15, 8, 4, 5, 13, 9, 0, 7, 11, 10 },
		{ 12, 6, 0, 4, 7, 3, 15, 1, 13, 9, 8, 11, 2, 14, 5, 10 },
		{ 8, 1, 7, 12, 11, 0, 10, 5, 9, 15, 6, 13, 14, 2, 3, 4 },
		{ 7, 15, 8, 2, 13, 6, 3, 12, 11, 0, 4, 10, 9, 5, 1, 14 },
		{ 9, 0, 4, 10, 1, 14, 15, 3, 12, 6, 5, 7, 11, 13, 8, 2 },
		{ 11, 5, 1, 14, 4, 12, 10, 0, 2, 7, 13, 3, 9, 15, 6, 8 },
		{ 8, 13, 10, 9, 11, 3, 15, 6, 0, 1, 2, 14, 12, 5, 4, 7 },
		{ 4, 5, 7, 2, 9, 14, 12, 13, 0, 3, 6, 11, 8, 1, 15, 10 },
		{ 11, 15, 14, 13, 1, 9, 10, 4, 3, 6, 2, 12, 7, 5, 8, 0 },
		{ 12, 9, 0, 6, 8, 3, 5, 14, 2, 4, 11, 7, 10, 1, 15, 13 },
		{ 3, 14, 9, 7, 12, 15, 0, 4, 1, 8, 5, 6, 11, 10, 2, 13 },
		{ 8, 4, 6, 1, 14, 12, 2, 15, 13, 10, 9, 5, 3, 7, 0, 11 },
		{ 6, 10, 1, 14, 15, 8, 3, 5, 13, 0, 2, 7, 4, 9, 11, 12 },
		{ 8, 11, 4, 6, 7, 3, 10, 9, 2, 12, 15, 13, 0, 1, 5, 14 },
		{ 10, 0, 2, 4, 5, 1, 6, 12, 11, 13, 9, 7, 15, 3, 14, 8 },
		{ 12, 5, 13, 11, 2, 10, 0, 9, 7, 8, 4, 3, 14, 6, 15, 1 },
		{ 10, 2, 8, 4, 15, 0, 1, 14, 11, 13, 3, 6, 9, 7, 5, 12 },
		{ 10, 8, 0, 12, 3, 7, 6, 2, 1, 14, 4, 11, 15, 13, 9, 5 },
		{ 14, 9, 12, 13, 15, 4, 8, 10, 0, 2, 1, 7, 3, 11, 5, 6 },
		{ 12, 11, 0, 8, 10, 2, 13, 15, 5, 4, 7, 3, 6, 9, 14, 1 },
		{ 13, 8, 14, 3, 9, 1, 0, 7, 15, 5, 4, 10, 12, 2, 6, 11 },
		{ 3, 15, 2, 5, 11, 6, 4, 7, 12, 9, 1, 0, 13, 14, 10, 8 },
		{ 5, 11, 6, 9, 4, 13, 12, 0, 8, 2, 15, 10, 1, 7, 3, 14 },
		{ 5, 0, 15, 8, 4, 6, 1, 14, 10, 11, 3, 9, 7, 12, 2, 13 },
		{ 15, 14, 6, 7, 10, 1, 0, 11, 12, 8, 4, 9, 2, 5, 13, 3 },
		{ 11, 14, 13, 1, 2, 3, 12, 4, 15, 7, 9, 5, 10, 6, 8, 0 },
		{ 6, 13, 3, 2, 11, 9, 5, 10, 1, 7, 12, 14, 8, 4, 0, 15 },
		{ 4, 6, 12, 0, 14, 2, 9, 13, 11, 8, 3, 15, 7, 10, 1, 5 },
		{ 8, 10, 9, 11, 14, 1, 7, 15, 13, 4, 0, 12, 6, 2, 5, 3 },
		{ 5, 2, 14, 0, 7, 8, 6, 3, 11, 12, 13, 15, 4, 10, 9, 1 },
		{ 7, 8, 3, 2, 10, 12, 4, 6, 11, 13, 5, 15, 0, 1, 9, 14 },
		{ 11, 6, 14, 12, 3, 5, 1, 15, 8, 0, 10, 13, 9, 7, 4, 2 },
		{ 7, 1, 2, 4, 8, 3, 6, 11, 10, 15, 0, 5, 14, 12, 13, 9 },
		{ 7, 3, 1, 13, 12, 10, 5, 2, 8, 0, 6, 11, 14, 15, 4, 9 },
		{ 6, 0, 5, 15, 1, 14, 4, 9, 2, 13, 8, 10, 11, 12, 7, 3 },
		{ 15, 1, 3, 12, 4, 0, 6, 5, 2, 8, 14, 9, 13, 10, 7, 11 },
		{ 5, 7, 0, 11, 12, 1, 9, 10, 15, 6, 2, 3, 8, 4, 13, 14 },
		{ 12, 15, 11, 10, 4, 5, 14, 0, 13, 7, 1, 2, 9, 8, 3, 6 },
		{ 6, 14, 10, 5, 15, 8, 7, 1, 3, 4, 2, 0, 12, 9, 11, 13 },
		{ 14, 13, 4, 11, 15, 8, 6, 9, 0, 7, 3, 1, 2, 10, 12, 5 },
		{ 14, 4, 0, 10, 6, 5, 1, 3, 9, 2, 13, 15, 12, 7, 8, 11 },
		{ 15, 10, 8, 3, 0, 6, 9, 5, 1, 14, 13, 11, 7, 2, 12, 4 },
		{ 0, 13, 2, 4, 12, 14, 6, 9, 15, 1, 10, 3, 11, 5, 8, 7 },
		{ 3, 14, 13, 6, 4, 15, 8, 9, 5, 12, 10, 0, 2, 7, 1, 11 },
		{ 0, 1, 9, 7, 11, 13, 5, 3, 14, 12, 4, 2, 8, 6, 10, 15 },
		{ 11, 0, 15, 8, 13, 12, 3, 5, 10, 1, 4, 6, 14, 9, 7, 2 },
		{ 13, 0, 9, 12, 11, 6, 3, 5, 15, 8, 1, 10, 4, 14, 2, 7 },
		{ 14, 10, 2, 1, 13, 9, 8, 11, 7, 3, 6, 12, 15, 5, 4, 0 },
		{ 12, 3, 9, 1, 4, 5, 10, 2, 6, 11, 15, 0, 14, 7, 13, 8 },
		{ 15, 8, 10, 7, 0, 12, 14, 1, 5, 9, 6, 3, 13, 11, 4, 2 },
		{ 4, 7, 13, 10, 1, 2, 9, 6, 12, 8, 14, 5, 3, 0, 11, 15 },
		{ 6, 0, 5, 10, 11, 12, 9, 2, 1, 7, 4, 3, 14, 8, 13, 15 },
		{ 9, 5, 11, 10, 13, 0, 2, 1, 8, 6, 14, 12, 4, 7, 3, 15 },
		{ 15, 2, 12, 11, 14, 13, 9, 5, 1, 3, 8, 7, 0, 10, 6, 4 },
		{ 11, 1, 7, 4, 10, 13, 3, 8, 9, 14, 0, 15, 6, 5, 2, 12 },
		{ 5, 4, 7, 1, 11, 12, 14, 15, 10, 13, 8, 6, 2, 0, 9, 3 },
		{ 9, 7, 5, 2, 14, 15, 12, 10, 11, 3, 6, 1, 8, 13, 0, 4 },
		{ 3, 2, 7, 9, 0, 15, 12, 4, 6, 11, 5, 14, 8, 13, 10, 1 },
		{ 13, 9, 14, 6, 12, 8, 1, 2, 3, 4, 0, 7, 5, 10, 11, 15 },
		{ 5, 7, 11, 8, 0, 14, 9, 13, 10, 12, 3, 15, 6, 1, 4, 2 },
		{ 4, 3, 6, 13, 7, 15, 9, 0, 10, 5, 8, 11, 2, 12, 1, 14 },
		{ 1, 7, 15, 14, 2, 6, 4, 9, 12, 11, 13, 3, 0, 8, 5, 10 },
		{ 9, 14, 5, 7, 8, 15, 1, 2, 10, 4, 13, 6, 12, 0, 11, 3 },
		{ 0, 11, 3, 12, 5, 2, 1, 9, 8, 10, 14, 15, 7, 4, 13, 6 },
		{ 7, 15, 4, 0, 10, 9, 2, 5, 12, 11, 13, 6, 1, 3, 14, 8 },
		{ 11, 4, 0, 8, 6, 10, 5, 13, 12, 7, 14, 3, 1, 2, 9, 15 }
	};
	for (int i = 0; i < 16; i++)
	{
		s.puzzle[i] = states[which][i];
		if (s.puzzle[i] == 0)
			s.blank = i;
	}
		
}

void BFS()
{
	Timer t;
	t.StartTimer();
	ClearFiles();
	RubiksCube c;
	RubiksState s;
	//uint64_t start1 = strtoll(argv[0], 0, 10);
	//GetInstanceFromStdin(s);
	GetKorfInstance(s, 0);
	//GetSuperFlip(s);
	hash128 start;
	start.parent = 20;
	start.edgeHash = c.GetEdgeHash(s);
	start.cornerHash = c.GetCornerHash(s);
	std::vector<hash128> states;
	states.push_back(start);
	WriteStatesToDisk(states, 0);

	// write goal to disk
	s.Reset();
	start.parent = 20;
	start.edgeHash = c.GetEdgeHash(s);
	start.cornerHash = c.GetCornerHash(s);
	states.clear();
	states.push_back(start);
	WriteStatesToDisk(states, 1);
	
	int depth = 2;
	bool done = false;
	while (!done)
	{
		Timer t2, t3;
		t2.StartTimer();
		t3.StartTimer();
		printf("***Starting layer: %d\n", depth); fflush(stdout);
		ExpandLayer(depth-2);
		printf("%1.2fs expanding\n", t2.EndTimer());  fflush(stdout);
		t2.StartTimer();
		done = DuplicateDetectLayer(depth);
		printf("%1.2fs dd\n", t2.EndTimer());
		printf("%1.2fs total elapsed at depth %d\n", t3.EndTimer(), depth);  fflush(stdout);
		depth++;
	}
	printf("%1.2fs elapsed; found solution at depth %d (cost %d)\n", t.EndTimer(), depth-1, depth-2);  fflush(stdout);
}

const char *GetFileName(int depth, int bucket)
{
	static char fname[255];
	sprintf(fname, "bfs-d%d-b%d.dat", depth, bucket);
	return fname;
}

void ClearFiles()
{
	for (int depth = 0; depth < 20; depth++)
	{
		for (int bucket = 0; bucket < kNumBuckets; bucket++)
			remove(GetFileName(depth, bucket));
	}
}

void ConvertToBucketHash(const hash128 &s, uint64_t &bucket, uint64_t &hash)
{
	static uint64_t maxEdgeRank = RubikEdge().getMaxSinglePlayerRank();
	bucket = s.cornerHash%kNumBuckets;
	hash   = (((s.cornerHash/kNumBuckets)*maxEdgeRank + s.edgeHash)<<5) + s.parent;
}

void ConvertBucketHashToState(uint64_t bucket, uint64_t hash, hash128 &s)
{
	static uint64_t maxEdgeRank = RubikEdge().getMaxSinglePlayerRank();
	
	s.parent = hash&0x1F;
	hash >>= 5;
	s.edgeHash = hash%maxEdgeRank;
	s.cornerHash = uint32_t(hash/maxEdgeRank);
	s.cornerHash = s.cornerHash*kNumBuckets + uint32_t(bucket);

}

void WriteStatesToDisk(std::vector<hash128> &states, int depth)
{
	RubiksCorner c;
	RubikEdge e;

	std::vector<FILE *> files;
	files.resize(kNumBuckets);
	std::vector<uint64_t> counts;
	counts.resize(kNumBuckets);
	
	// 40 bits for edges
	// 27 bits for corners
	// 67 bits
	// 512 buckets (9 bits)
	// 58 bits total
	// 6 bits for extra information (parent)
	for (auto &s : states)
	{
		uint64_t bucket, hash;
		ConvertToBucketHash(s, bucket, hash);
//		uint64_t bucket = s.cornerHash%kNumBuckets;
//		uint64_t hash   = (s.cornerHash/kNumBuckets)*e.getMaxSinglePlayerRank() + s.edgeHash;
		if (files[bucket] == 0)
		{
			files[bucket] = fopen(GetFileName(depth, (int)bucket), "a");
			if (files[bucket] == 0)
			{
				printf("Can't open file %s\n", GetFileName(depth, (int)bucket));
			}
		}
//		if (depth < 2)
//			printf("%2d): Writing %llu %llu\n", depth, bucket, hash);
		fwrite(&hash, sizeof(uint64_t), 1, files[bucket]);
		counts[bucket]++;
	}
	for (int x = 0; x < files.size(); x++)
	{
		FILE *f = files[x];
		if (f)
		{
			fclose(f);
			//printf("%llu states written to depth %d bucket %d [%s]\n", counts[x], depth, x, GetFileName(depth, (int)x));
		}
	}
}

void ExpandLayer(int depth)
{
	RubikEdge e;
	RubiksCube c;
	RubiksState s;
	std::vector<RubiksAction> acts;
	std::vector<hash128> nextLevel;
	hash128 parent, child;
	uint64_t total = 0;
	for (int x = 0; x < kNumBuckets; x++)
	{
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;
		//printf("-Expanding depth %d bucket %d [%s]\n", depth, x, GetFileName(depth, x));
		uint64_t next;
		uint64_t count = 0;
		while (fread(&next, sizeof(uint64_t), 1, f) == 1)
		{
			total++;
			count++;
			ConvertBucketHashToState(x, next, parent);
//			uint64_t cornerRank;
//			uint64_t edgeRank = next%e.getMaxSinglePlayerRank();
//			cornerRank = next/e.getMaxSinglePlayerRank();
//			cornerRank = cornerRank*kNumBuckets + x;
			//printf("%2d): Expanding %llu %llu\n", depth, x, next);
			c.GetStateFromHash(parent.cornerHash, parent.edgeHash, s);
			if (parent.parent < 20)
				c.GetPrunedActions(s, parent.parent, acts);
			else
				c.GetActions(s, acts);
			for (int a = 0; a < acts.size(); a++)
			{
				c.ApplyAction(s, acts[a]);
//				int toParent = acts[a];
//				c.InvertAction(toParent);
				child.parent = acts[a];
				child.edgeHash = c.GetEdgeHash(s);
				child.cornerHash = c.GetCornerHash(s);
				c.UndoAction(s, acts[a]);
				nextLevel.push_back(child);
			}
		}
		fclose(f);
		//printf("-Read %llu states from depth %d bucket %d\n", count, depth, x);
		WriteStatesToDisk(nextLevel, depth+2);
		nextLevel.clear();
	}
	printf("%llu states at depth %d\n", total, depth);
}

bool DuplicateDetectLayer(int depth)
{
	const int bufferSize = 128;
	
	double m0Dups = 0;
	double m2Dups = 0;
	double m4Dups = 0;
	double fDups = 0;
	double writeTime = 0;
	std::vector<uint64_t> values;
	values.resize(bufferSize);
	Timer t;
	uint64_t count = 0;
	bool dups = false;
	std::unordered_map<uint64_t, uint8_t> map;
	uint64_t removed0 = 0, removed2 = 0;
	for (int x = 0; x < kNumBuckets; x++)
	{
		t.StartTimer();
		FILE *f = fopen(GetFileName(depth, x), "r");
		if (f == 0)
			continue;

		map.clear();
		count = 0;
		uint64_t numRead;
		while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
		{
			for (int x = 0; x < numRead; x++)
				map[values[x]>>5] = values[x]&0x1F;
			count++;
		}
		fclose(f);
		m0Dups += t.EndTimer();
		//printf("Read %llu states from depth %d bucket %d [%s]\n", count, depth, x, GetFileName(depth, x));
		
		t.StartTimer();
		f = fopen(GetFileName(depth-2, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					//printf("Looking for duplicate %d %llu\n", x, next);
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						//printf("Removing duplicate %d %llu\n", x, loc->first);
						removed0++;
						map.erase(loc);
					}
				}
			}
			fclose(f);
		}
		m2Dups += t.EndTimer();

		t.StartTimer();
		f = fopen(GetFileName(depth-4, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						removed2++;
						map.erase(loc);
					}
				}
			}
			fclose(f);
		}
		m4Dups += t.EndTimer();

		t.StartTimer();
		f = fopen(GetFileName(depth-1, x), "r");
		if (f != 0)
		{
			while ((numRead = fread(&values[0], sizeof(uint64_t), bufferSize, f)) > 0)
			{
				for (int x = 0; x < numRead; x++)
				{
					auto loc = map.find(values[x]>>5);
					if (loc != map.end())
					{
						//printf("Found duplicate!\n");
						dups = true;
					}
				}
			}
			fclose(f);
		}
		fDups += t.EndTimer();
		
		t.StartTimer();
		count = 0;
		f = fopen(GetFileName(depth, x), "w");
		if (f == 0)
		{
			printf("Error with %s\n", GetFileName(depth, x));
			exit(0);
		}
		values.resize(0);
		for (auto val : map)
		{
			//if (val.second)
			{
				values.push_back((val.first<<5)|(val.second));
				count++;
				//printf("%2d): Writing %llu %llu\n", depth, x, val.first);
				
				if (values.size() >= bufferSize)
				{
					fwrite(&(values[0]), sizeof(uint64_t), values.size(), f);
					values.resize(0);
				}
			}
		}
		if (values.size() > 0)
			fwrite(&(values[0]), sizeof(uint64_t), values.size(), f);
		//printf("Wrote %llu states to depth %d bucket %d [%s]\n", count, depth, x, GetFileName(depth, x));
		fclose(f);
		writeTime += t.EndTimer();
	}
	printf("%1.2f dup vs 0, %1.2f dup vs -2, %1.2f dup vs -4, %1.2f dup vs other frontier, %1.2f write; %llu dups at -2, %llu at -4\n",
		   m0Dups, m2Dups, m4Dups, fDups, writeTime, removed0, removed2);
	return dups;
}













int GetNextMove(char *input, int &base)
{
	int used = 0;
	if (isdigit(input[0])) // this is our move notation - numeric
	{
		int curr = 0;
		base = input[curr]-'0';
		curr++;
		if (isdigit(input[curr]))
		{
			base = base*10+input[curr]-'0';
			curr++;
		}
		while (!isdigit(input[curr]) && input[curr] != '\n' && input[curr] != 0)
		{
			curr++;
		}
		return curr;
	}
	switch (input[0])
	{
		case 'F': used = 1; base = 2*3; break;
		case 'B': used = 1; base = 3*3; break;
		case 'L': used = 1; base = 4*3; break;
		case 'R': used = 1; base = 5*3; break;
		case 'U': used = 1; base = 0*3; break;
		case 'D': used = 1; base = 1*3; break;
		default: break;
	}
	if (used == 0)
		return 0;
	if (input[0] != 'U')
	{
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 2; used++; break;
				case '-': base += 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	else {
		base++;
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 1; used++; break;
				case '-': base -= 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	return used;
}

void GetDepth20(RubiksState &start, int which)
{
	const int maxStrLength = 1024;
	char string[10][maxStrLength] = //"U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";
	{
		"B2 L B2 R- F- U- B- L D- F- L U L2 B2 L- D2 B2 D2 R2 B2",
		"R U2 R D2 R2 B2 L- D- B3 F U B- R- U2 L- D R2 F- U2 L2",
		"D2 R2 F2 D2 F2 D2 R- F2 D- L2 R B L- F U R- B F2 R2 F-",
		"D- F- U B2 R2 F R- U2 B- L D F R D2 R2 L2 D- R2 F2 D-",
		"U2 R2 F2 D- U F2 U2 B U B- R U- F L B R- F L2 D- B",
		"D B2 D- B2 R2 D- R2 U L R- D B- D R F- D2 R2 U- F- R",
		"B D- L- F- L F B U- D2 F- R2 B- U F2 R- L U2 R2 F2 B2",
		"U2 L- U2 F2 L- R D2 L2 B- D2 L F- R- U- L U2 F- D- R B",
		"F- L B2 R U- B- L U2 D3 F L- R2 U2 D2 B2 R2 D R2 L2 F2",
		"U2 R2 D2 B U2 B- F D- B- R- D U2 B2 F2 R- D- B U- F- R2"
	};

	RubiksCube c;

	start.Reset();
	
	int index = 0;
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[which][index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}


void GetSuperFlip(RubiksState &start)
{
	RubiksCube c;
	const int maxStrLength = 1024;
	char string[maxStrLength] = "U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";

	start.Reset();
	
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetKorfInstance(RubiksState &start, int which)
{
	const int maxStrLength = 1024;
	assert(which >= 0 && which < 10);
	RubiksCube c;
	char instances[10][maxStrLength] =
	{
		"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2 B- U  R- D2 F  R- B  L R  U- B- R2 F- L2 R  F  R2 B L- F- D- F2 U2 R  U- L  D  F2 B- R- D- L2 B- L- B2 L- D2 B2 D- B  D  R- B  D  L- B- R  F- L- F- R2 D2 L2 B- L2 B2 U  L2",
		"B- R2 B  D  B- L  B  L2 F2 R F2 D- L2 U2 L- U  L- U2 B- L- R- U  D  L- B2 D  R- U  F  D2 F  B  U  B2 L2 D2 R- B2 L- R2 U2 D2 F2 D  R2 D2 B- U- D  F- R  B2 D  R2 F  L- B  L2 R- U2 L  F2 B- D- F- B- L2 D  B2 U- D  F2 U  L2 D  L- D- R2 D- B2 U- L2 U  B- L- U- F- L- R- B- U- R  B2 U2 B  R- B- R2 F  R-",
		"L- R  F- L  R2 F2 D- L2 D  B2 R2 D- F- L- F  R  F2 U  L- B2 D- R- U- R  D  F  R  D  B2 U- F- L2 F- B  U- R  F- U  F  D- L2 R- F- B  L2 B2 D- R- B  L B  D- R  U- R2 D2 F  R  U2 B2 D2 B  R- F- L2 D2 L2 R  D  L- B2 U  F2 R  F  L  U  D  L- B2 L2 B2 D- L  D2 B- U- B- U2 B L  D  B- L- U2 L- R  D- R  B2",
		"L- B2 U2 R- D  F  U  F2 D- F U  D- R- B  R  U- R2 B  R  F D  R2 F- R- B2 R- D- R2 U- F- R  D  F- R  U- F  B  U- D2 B- L  D2 L2 B- U2 L2 F2 L  D2 B D- L  R2 B2 U2 F2 B- U2 F  D2 L2 U2 F2 L  R- U- R- D  F  L2 F2 L- R  U- L2 U  R  D  F  R- F- D- L- R2 U2 F  R- B2 D  B2 D2 B2 R  F- L- D  B- U  L2 B-",
		"B2 L- F- U2 R2 D  R2 B  U2 R B- L  R  F  R- D- R- D2 F2 R2 B  L2 D- B2 D2 L  F- R  B- R2 B2 D  B- U  R- D- L2 B2 L2 R2 B- U2 D- R2 B  U- B- R- D  L B- L  R2 D- B  L2 D2 F2 B- U2 B  D- F- B  L2 U  F- U  F- L U  R  U2 D- B- U2 D  F- L  F2 B2 L  U  B- D2 B2 L- D- L2 B- D2 F  U2 R  D2 L2 D  B- L2 R-",
		"B- R2 B  R  U- D- R2 D  B  L2 B2 U- F- B  D2 R- U  F  L  F U2 B  L  D- L- R2 B  D  R- F- B- D  L- B- L2 R- U2 D  B  R- D2 B- U2 B- L- R2 F  L  U  L- R  B- R2 D2 R2 B- U2 F- U  L D- F  B- R2 D2 L  B  L  U2 B2 R  D- B- R2 B2 U  F2 R2 U- B- R2 F2 U2 F2 B- D  F  U- F2 B R  D  R- U- L- R  U2 D- R2 F",
		"L- R- B  L- D- L  U2 B2 D- F2 D  B2 R- F- R  U2 B2 D2 L- B- D  R2 D2 R  D- R2 F- R  U  B- U2 B2 D- R- D2 F  U  D- F- U2 L2 U  F2 R- B- U- L  B- R- U- L- U2 B2 D  B- R  B  D  B2 R- U2 D- R- F  L- F- D- B  L2 R- B  R- B- D  R  U- R2 B- D2 F- R  F  L- U  L2 B2 R2 F- U- F D  B- L- F2 B  L2 U2 D- L2 B2",
		"U  F  B2 L  F2 L- D2 B2 L2 U- R  D2 L2 U- D  F  U2 L2 B- U- B- R2 B2 R  U  R2 D- B2 R  B2 L2 U- R  D  L- R2 U2 R- D  B L2 R- B2 U2 L  U2 R  F2 U  B2 L2 R2 D- F  R2 D- L2 U- R  B2 D2 R- U2 L- B- L- F- L  R  F L- B- D  L- R2 F2 U- F  L- U2 B  U  R2 F  U  R- B  D  B  R- B- L- B2 R- F  L- B2 L- F2 D2",
		"L  F2 L2 B2 D  B  R  D- R- F2 U2 D2 F2 B- U- L- R2 B  U2 R F- D2 B2 U- R- F  L- U2 R- B L2 R2 B2 L2 R- B- D  F2 L  U2 D  L- B2 L  D- R2 D  B2 U  R- F2 U- R  F2 R2 B  U2 D2 R- U2 L2 R- D2 F2 R2 B- U  B  U  F D- F  D2 R- B- U2 B2 L  D  B- L- B2 D  B2 D- B  D- F- B- L- U  B2 D2 B- D- F  D- L  F  R",
		"B2 D- B- U- R- D- B- U2 L- R- B2 U2 B2 L- U  B- D  F  L2 F2 D  F- L- D- B2 L- U2 F  B2 R2 D  L- U  D2 F  B- L2 B  R- U B- L  B2 D  F  R  U- D- F  R2 U2 L- B2 L- R- D- L2 R- F2 D L- D  B2 D  L  B- R- D  B- L2 B2 D2 F  B2 U2 R- D- L- B2 R- D- L2 F- D- R  U  F  L2 D- R- U- L2 B  U- F2 U  B- D  F2 D2"
	};
	
	char *string = instances[which];
	start.Reset();
	
	//	if (result == 0)
	//	{
	//		printf("No more entries found; exiting.\n");
	//		exit(0);
	//	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetInstanceFromStdin(RubiksState &start)
{
	RubiksCube c;
	
	const int maxStrLength = 1024;
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B R- F- B R-";
//	char string[maxStrLength] =
//	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2";
//	char string[maxStrLength] = // length 18
//	"B2 D- B- U- R- D- B- U2 L- R- B2 U2 B2 L- U  B- D  F  L2 F2 D  F- L- D- B2 L- U2 F  B2 R2 D  L- U  D2 F  B- L2 B  R- U B- L  B2 D  F  R  U- D- F  R2 U2 L- B2 L- R- D- L2 R- F2 D L- D  B2 D  L  B- R- D  B- L2 B2 D2 F  B2 U2 R- D- L- B2 R- D- L2 F- D- R  U  F  L2 D- R- U- L2 B  U- F2 U  B- D  F2 D2";
	char string[maxStrLength] = // length 16
	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2 B- U  R- D2 F  R- B  L R  U- B- R2 F- L2 R  F  R2 B L- F- D- F2 U2 R  U- L  D  F2 B- R- D- L2 B- L- B2 L- D2 B2 D- B  D  R- B  D  L- B- R  F- L- F- R2 D2 L2 B- L2 B2 U  L2";
	// 	const char *result = fgets(string, maxStrLength, stdin);

	start.Reset();
	
//	if (result == 0)
//	{
//		printf("No more entries found; exiting.\n");
//		exit(0);
//	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}


void TestPruning(int depth, int bucket)
{
	RubikEdge e;
	RubiksCube c;
	RubiksState s;
	std::vector<uint64_t> depths(17);
	int count = 0;
	FILE *f = fopen(GetFileName(depth, bucket), "r");
	if (f == 0)
	{
		printf("Unable to open '%s'; aborting!\n", GetFileName(depth, bucket));
		return;
	}
	uint64_t next;
	while (fread(&next, sizeof(uint64_t), 1, f) == 1)
	{
		if (++count > 1000)
		{
			break;
		}
		printf("%d\r", count); fflush(stdout);
		uint64_t cornerRank;
		uint64_t edgeRank = next%e.getMaxSinglePlayerRank();
		cornerRank = next/e.getMaxSinglePlayerRank();
		cornerRank = cornerRank*kNumBuckets + bucket;
		//printf("%2d): Expanding %llu %llu\n", depth, x, next);
		c.GetStateFromHash(cornerRank, edgeRank, s);
		
		depths[c.Edge12PDBDist(s)]++;
	}
	fclose(f);
	printf("\n\n");
	for (int x = 0; x < depths.size(); x++)
	{
		printf("%d\t%llu\n", x, depths[x]);
	}
}
