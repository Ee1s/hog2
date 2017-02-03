//
//	NERO.h
//	This file derived from MM.h by Nathan Sturtevant
//	The following is the original claim
//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/27/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef NERO_H
#define NERO_H

#include "BDOpenClosed.h"
#include "FPUtil.h"


#define EPSILON 1

using std::cout;
//low g -> low f
template <class state>
struct NEROCompare0 {
	bool operator()(const BDOpenClosedData<state> &i1, const BDOpenClosedData<state> &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;

		if (fequal(i1.g, i2.g))
		{
			return (fgreater(f1, f2)); //equal g, low f over high
		}
		return (fgreater(i1.g, i2.g)); // low g over high
	}
};

template <class state>
struct NEROCompare1 {
	bool operator()(const BDOpenClosedData<state> &i1, const BDOpenClosedData<state> &i2) const
	{
		double f1 = i1.g + i1.h;
		double f2 = i2.g + i2.h;

		if (fequal(f1, f2))
		{
			//return (fgreater(i1.g, i2.g)); // low g-cost over high
		    return (fless(i1.g, i2.g)); // high g-cost over low
		}
		return (fgreater(f1, f2)); // low f over high
	}
};

template <class state, class action, class environment,  class priorityQueue = BDOpenClosed<state, NEROCompare0<state>, NEROCompare1<state>>>
class NERO {
public:
	NERO(priorityQueue f, priorityQueue b)
		:forwardQueue(f),backwardQueue(b)
	{ forwardHeuristic = 0; backwardHeuristic = 0; env = 0; ResetNodeCount(); 
	}
	virtual ~NERO() {}
	void GetPath(environment *env, const state& from, const state& to,
				 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool InitializeSearch(environment *env, const state& from, const state& to,
						  Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);
	bool ExpandAPair(std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	

	
	
	virtual const char *GetName() { return "NERO"; }
	
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	
//	bool GetClosedListGCost(const state &val, double &gCost) const;
//	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
//	inline const AStarOpenClosedData<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumForwardItems() { return forwardQueue.size(); }
	inline const BDOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }
	inline const int GetNumBackwardItems() { return backwardQueue.size(); }
	inline const BDOpenClosedData<state> &GetBackwardItem(unsigned int which) { return backwardQueue.Lookat(which); }
//	bool HaveExpandedState(const state &val)
//	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
//	
	void SetForwardHeuristic(Heuristic<state> *h) { forwardHeuristic = h; }
	void SetBackwardHeuristic(Heuristic<state> *h) { backwardHeuristic = h; }
	stateLocation GetNodeForwardLocation(const state &s) 
	{
		uint64_t childID;
		auto l = forwardQueue.Lookup(env->GetStateHash(s), childID);
			return l;
	}
	stateLocation GetNodeBackwardLocation(const state &s)
	{
		uint64_t childID;
		return backwardQueue.Lookup(env->GetStateHash(s), childID);
	}
	double GetNodeForwardG(const state& s)
	{

		uint64_t childID;
		auto l = forwardQueue.Lookup(env->GetStateHash(s), childID);
		if (l != kUnseen)
			return forwardQueue.Lookat(childID).g;
		return -1;
	}
	double GetNodeBackwardG(const state& s)
	{

		uint64_t childID;
		auto l = backwardQueue.Lookup(env->GetStateHash(s), childID);
		if (l != kUnseen)
			return backwardQueue.Lookat(childID).g;
		return -1;
	}
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	double GetSolutionCost() const { return currentCost; }
	//void FullBPMX(uint64_t nodeID, int distance);
	
	void OpenGLDraw() const;
	
//	void SetWeight(double w) {weight = w;}
private:
	void OpenGLDraw(const priorityQueue &queue) const;
	
	void Expand(priorityQueue &current,
				priorityQueue &opposite,
				Heuristic<state> *heuristic, const state &target);
	//direction ==0 forward; 1 backward
	//void Expand(int direction);
	uint64_t nodesTouched, nodesExpanded;
	state middleNode;
	double currentCost;
	std::vector<state> neighbors;
	environment *env;
	

	priorityQueue forwardQueue, backwardQueue;
	//priorityQueue2 forwardQueue, backwardQueue;

	state goal, start;

	Heuristic<state> *forwardHeuristic;
	Heuristic<state> *backwardHeuristic;

	//keep track of whether we expand a node or put it back to open
	bool expand;

	double currentPr;


};

template <class state, class action, class environment, class priorityQueue>
void NERO<state, action, environment, priorityQueue>::GetPath(environment *env, const state& from, const state& to,
			 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath)
{
	if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
		return;
	
	while (!ExpandAPair(thePath))
	{ }
	//printf("get a path\n");
}

template <class state, class action, class environment, class priorityQueue>
bool NERO<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state& from, const state& to,
																	 Heuristic<state> *forward, Heuristic<state> *backward,
																	 std::vector<state> &thePath)
{
	this->env = env;
	forwardHeuristic = forward;
	backwardHeuristic = backward;
	currentCost = DBL_MAX;
	forwardQueue.Reset();
	backwardQueue.Reset();
	ResetNodeCount();
	thePath.resize(0);
	start = from;
	goal = to;
	if (start == goal)
		return false;

	forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal),1);
	backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, forwardHeuristic->HCost(goal, start),1);



	
	return true;
}

template <class state, class action, class environment, class priorityQueue>
bool NERO<state, action, environment, priorityQueue>::ExpandAPair(std::vector<state> &thePath)
{
	assert(forwardQueue.Validate0());
	assert(forwardQueue.Validate1());
	assert(backwardQueue.Validate0());
	assert(backwardQueue.Validate1());

	//std::cout << "forwardQueue.OpenSize() " << forwardQueue.OpenSize() << " backwardQueue.OpenSize() "
	//	<< backwardQueue.OpenSize() << "\n";
	if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
	{
		printf("No more pairs, terminate!\n");
		return true;
	}
	//if (forwardQueue.OpenSize() == 0 && backwardQueue.OpenSize() == 0)
	//{
	//	printf("No more pairs, terminate!\n");
	//	return true;
	//}
	//else if (forwardQueue.OpenSize() == 0)
	//{
	//	//printf("expand backward only\n");
	//	uint64_t nextIDBackward;
	//	if (backwardQueue.OpenReadySize() == 0)
	//	{
	//		nextIDBackward = backwardQueue.PutToReady();

	//		auto iB = backwardQueue.Lookat(nextIDBackward);
	//		if (!fless(iB.g + iB.h, currentCost))
	//		{
	//			printf("Current Backward f-cost: %1.3f, current Best Solution: %1.3f, can terminate\n", iB.g + iB.h, currentCost);
	//			return true;
	//		}
	//	}


	//	Expand(backwardQueue, forwardQueue, backwardHeuristic, start);
	//	return false;
	//}
	//else if (backwardQueue.OpenSize()==0)
	//{
	//	//printf("expand forward only\n");
	//	uint64_t nextIDForward;
	//	if (forwardQueue.OpenReadySize() == 0)
	//	{
	//		nextIDForward = forwardQueue.PutToReady();
	//		auto iF = forwardQueue.Lookat(nextIDForward);
	//		if (!fless(iF.g + iF.h, currentCost))
	//		{
	//			printf("Current Forward f-cost: %1.3f, current Best Solution: %1.3f, can terminate\n", iF.g + iF.h, currentCost);
	//			return true;
	//		}
	//	}


	//	Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);
	//	return false;
	//}
	else
	{

		uint64_t nextIDForward;
		uint64_t nextIDBackward;
		BDOpenClosedData<state> iFReady, iBReady, iFWaiting, iBWaiting;

		assert(forwardQueue.Validate0());
		assert(forwardQueue.Validate1());
		assert(backwardQueue.Validate0());
		assert(backwardQueue.Validate1());

		//first of all, just put something into them
		std::cout << "1 forwardQueue.OpenWaitngSize(): " << forwardQueue.OpenWaitingSize()
			<< "backwardQueue.OpenWaitngSize(): " << backwardQueue.OpenWaitingSize()<<"\n";
		if (forwardQueue.OpenReadySize() == 0)
			forwardQueue.PutToReady();
		if (backwardQueue.OpenReadySize() == 0)
			backwardQueue.PutToReady();

		std::cout << "2 forwardQueue.OpenWaitngSize(): " << forwardQueue.OpenWaitingSize()
			<< "backwardQueue.OpenWaitngSize(): " << backwardQueue.OpenWaitingSize() << "\n";
		assert(forwardQueue.Validate0());
		assert(forwardQueue.Validate1());
		assert(backwardQueue.Validate0());
		assert(backwardQueue.Validate1());

		iFReady = forwardQueue.Lookat(forwardQueue.Peek(kOpenReady));
		iBReady = backwardQueue.Lookat(backwardQueue.Peek(kOpenReady));

		if (forwardQueue.OpenWaitingSize() != 0 || backwardQueue.OpenWaitingSize() != 0)
		{
			double fBound, gBound;

			gBound = iFReady.g + iBReady.g + EPSILON;
			if (forwardQueue.OpenWaitingSize() == 0)
			{
				iBWaiting = backwardQueue.Lookat(backwardQueue.Peek(kOpenWaiting));
				fBound = iBWaiting.g + iBWaiting.h;
			}
			else if (backwardQueue.OpenWaitingSize() == 0)
			{
				iFWaiting = forwardQueue.Lookat(forwardQueue.Peek(kOpenWaiting));
				fBound = iFWaiting.g + iFWaiting.h;
			}
			else//forwardQueue.OpenWaitingSize() > 0 && backwardQueue.OpenWaitingSize() > 0
			{
				iFWaiting = forwardQueue.Lookat(forwardQueue.Peek(kOpenWaiting));
				iBWaiting = backwardQueue.Lookat(backwardQueue.Peek(kOpenWaiting));
				fBound = std::min(iFWaiting.g + iFWaiting.h, iBWaiting.g + iBWaiting.h);
			}

			while (fgreater(gBound, fBound))
			{
				assert(forwardQueue.Validate0());
				assert(forwardQueue.Validate1());
				assert(backwardQueue.Validate0());
				assert(backwardQueue.Validate1());
				if (forwardQueue.OpenWaitingSize() == 0)
					backwardQueue.PutToReady();
				else if (backwardQueue.OpenWaitingSize() == 0)
					forwardQueue.PutToReady();
				else
				{
					nextIDForward = forwardQueue.Peek(kOpenWaiting);
					nextIDBackward = backwardQueue.Peek(kOpenWaiting);
					auto iF = forwardQueue.Lookat(nextIDForward);
					auto iB = backwardQueue.Lookat(nextIDBackward);
					if (fless(iF.g + iF.h, iB.g + iB.h))
						forwardQueue.PutToReady();
					else
						backwardQueue.PutToReady();
				}
				assert(forwardQueue.Validate0());
				assert(forwardQueue.Validate1());
				assert(backwardQueue.Validate0());
				assert(backwardQueue.Validate1());



				if (forwardQueue.OpenWaitingSize() == 0 && backwardQueue.OpenWaitingSize() == 0)
					break;
				else if (forwardQueue.OpenWaitingSize() == 0)
				{
					iBWaiting = backwardQueue.Lookat(backwardQueue.Peek(kOpenWaiting));

					fBound = iBWaiting.g + iBWaiting.h;
				}
				else if (backwardQueue.OpenWaitingSize() == 0)
				{
					iFWaiting = forwardQueue.Lookat(forwardQueue.Peek(kOpenWaiting));
					fBound = iFWaiting.g + iFWaiting.h;
				}
				else //forwardQueue.OpenWaitingSize() > 0 && backwardQueue.OpenWaitingSize() > 0
				{
					iFWaiting = forwardQueue.Lookat(forwardQueue.Peek(kOpenWaiting));
					iBWaiting = backwardQueue.Lookat(backwardQueue.Peek(kOpenWaiting));

					fBound = std::min(iFWaiting.g + iFWaiting.h, iBWaiting.g + iBWaiting.h);
					//TODO epsilon
				}
				iFReady = forwardQueue.Lookat(forwardQueue.Peek(kOpenReady));
				iBReady = backwardQueue.Lookat(backwardQueue.Peek(kOpenReady));
				gBound = iFReady.g + iBReady.g + EPSILON;
			}

			iFReady = forwardQueue.Lookat(forwardQueue.Peek(kOpenReady));
			iBReady = backwardQueue.Lookat(backwardQueue.Peek(kOpenReady));
		}
		if (iFReady.data == iBReady.data)
		{
			printf("same nodes, can terminate, %1.3f\n", currentCost);
			std::cout << "fdata" << iFReady.data << "bdata" << iBReady.data << "\n";
			return true;
		}


 		double minPr = std::max(iFReady.g + iFReady.h, iBReady.g + iBReady.h);
		minPr = std::max(minPr, iFReady.g + iBReady.g + EPSILON);
		
		std::cout << "fdata" << iFReady.data << "bdata" << iBReady.data << "\n";
		double fb = std::max(iFReady.g + iFReady.h, iBReady.g + iBReady.h);
		double gb = iFReady.g + iBReady.g + EPSILON;
		if (fless(fb, gb))
		{
			printf("lb = gbound");
		}
		else
		{
			printf("lb = fbound");
		}
		printf(" ,fb = %1.3f, gb = %1.3f, lb = %1.3f\n",fb,gb,minPr);
		//TODO epsilon

		//printf("Current Priority: %1.3f, current Best Solution: %1.3f\n", minPr, currentCost);


		if (!fless(minPr, currentCost))
		{
			printf("Current Priority: %1.3f, current Best Solution: %1.3f, can terminate\n", minPr, currentCost);
			return true;
		}
		else
		{
			Expand(forwardQueue, backwardQueue, forwardHeuristic, goal);
			Expand(backwardQueue, forwardQueue, backwardHeuristic, start);
		}
		return false;
	}
	

}




template <class state, class action, class environment, class priorityQueue>
bool NERO<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
	return false;
}


template <class state, class action, class environment, class priorityQueue>
void NERO<state, action, environment, priorityQueue>::Expand(priorityQueue &current,
														   priorityQueue &opposite,
														   Heuristic<state> *heuristic, const state &target)
{

	uint64_t nextID = current.Peek(kOpenReady);

	nextID = current.Close();

	//this can happen when we expand a single node instead of a pair
	if (!fless(current.Lookup(nextID).g + current.Lookup(nextID).h, currentCost))
		return;

	nodesExpanded++;

	env->GetSuccessors(current.Lookup(nextID).data, neighbors);
	for (auto &succ : neighbors)
	{
		nodesTouched++;
		uint64_t childID;
		auto loc = current.Lookup(env->GetStateHash(succ), childID);
		switch (loc)
		{
			case kClosed: // ignore
				break;
			case kOpenReady: // update cost if needed
			case kOpenWaiting:
			{
				double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
				if (fless(current.Lookup(nextID).g+edgeCost, current.Lookup(childID).g))
				{
					double oldGCost = current.Lookup(childID).g;
					current.Lookup(childID).parentID = nextID;
					current.Lookup(childID).g = current.Lookup(nextID).g+edgeCost;
					current.KeyChanged(childID);
					

					//if (current.Lookup(childID).pathCost >0)
					//{
					//	current.Lookup(childID).pathCost = current.Lookup(childID).pathCost - oldGCost
					//		+ current.Lookup(nextID).g + edgeCost;

					//	if (fless(current.Lookup(childID).pathCost, currentCost))
					//	{
					//		printf("NERO Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
					//			current.Lookup(nextID).g + edgeCost,
					//			current.Lookup(childID).pathCost - current.Lookup(nextID).g + edgeCost,
					//			current.Lookup(childID).pathCost,
					//			nodesExpanded);
					//		currentCost = current.Lookup(childID).pathCost;
					//	}
					//	
					//	
					//}

					// TODO: check if we improved the current solution?
					uint64_t reverseLoc;
					auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
							printf("NERO Potential updated solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
								   current.Lookup(nextID).g+edgeCost,
								   opposite.Lookup(reverseLoc).g,
								   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g,
								nodesExpanded);
							currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;

							middleNode = succ;
						}
					}
					else if (loc == kClosed)
					{
						//current.Lookup(childID).h = opposite.Lookup(reverseLoc).g;
						//current.Lookup(childID).g = 100000;
						//current.KeyChanged(childID);
						current.Remove(childID);
					}
				}
			}
				break;
			case kUnseen:
			{
				uint64_t reverseLoc;
				auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
				if (loc == kClosed)// then 
				{
					break;			//do nothing. do not put this node to open
				}

				else//loc == kUnseen
				{
					double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
					//if(fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target),currentPr))
					//	current.AddOpenNode(succ,
					//		env->GetStateHash(succ),
					//		current.Lookup(nextID).g + edgeCost,
					//		heuristic->HCost(succ, target),
					//		nextID,0);
					//else

					if(fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target) , currentCost))
					current.AddOpenNode(succ,
						env->GetStateHash(succ),
						current.Lookup(nextID).g + edgeCost,
						heuristic->HCost(succ, target),
						nextID, kOpenWaiting);
					if (loc == kOpenReady || loc == kOpenWaiting)
					{
						double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
						//if (opposite.Lookup(reverseLoc).pathCost == 0 ||
						//	(opposite.Lookup(reverseLoc).pathCost > 0
						//		&& fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, opposite.Lookup(reverseLoc).pathCost)))
						//{
						//	opposite.Lookup(reverseLoc).pathCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
						//}
						//// check for solution
						if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
						{
							// TODO: store current solution
							printf("NERO Potential solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
								current.Lookup(nextID).g + edgeCost,
								opposite.Lookup(reverseLoc).g,
								current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g,
								nodesExpanded);
							currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;

							middleNode = succ;
						}

					}
				}

				//else if (loc == kOpenReady || loc == kOpenWaiting)
				//{
				//	double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
				//	if (opposite.Lookup(reverseLoc).pathCost == 0 ||
				//		(opposite.Lookup(reverseLoc).pathCost > 0
				//			&& fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, opposite.Lookup(reverseLoc).pathCost)))
				//	{
				//		opposite.Lookup(reverseLoc).pathCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
				//	}
				//	//// check for solution
				//	if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
				//	{
				//		// TODO: store current solution
				//		printf("NERO Potential solution found, cost: %1.2f + %1.2f = %1.2f (%llu nodes)\n",
				//			current.Lookup(nextID).g + edgeCost,
				//			opposite.Lookup(reverseLoc).g,
				//			current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g,
				//			nodesExpanded);
				//		currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;

				//		middleNode = succ;
				//	}

				//}
				//else//loc == kUnseen
				//{
				//	double edgeCost = env->GCost(current.Lookup(nextID).data, succ);
				//	//if(fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target),currentPr))
				//	//	current.AddOpenNode(succ,
				//	//		env->GetStateHash(succ),
				//	//		current.Lookup(nextID).g + edgeCost,
				//	//		heuristic->HCost(succ, target),
				//	//		nextID,0);
				//	//else

				//	if (fless(current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target), currentCost))
				//		current.AddOpenNode(succ,
				//			env->GetStateHash(succ),
				//			current.Lookup(nextID).g + edgeCost,
				//			heuristic->HCost(succ, target),
				//			nextID, kOpenWaiting);
				//	
				//}
			}
			break;
		}
	}
}



template <class state, class action, class environment, class priorityQueue>
void NERO<state, action, environment, priorityQueue>::OpenGLDraw() const
{
	OpenGLDraw(forwardQueue);
	OpenGLDraw(backwardQueue);
}

template <class state, class action, class environment, class priorityQueue>
void NERO<state, action, environment, priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
{
	double transparency = 0.9;
	if (queue.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (queue.OpenReadySize() > 0)
	{
		top = queue.Peek(kOpenReady);
	}
	for (unsigned int x = 0; x < queue.size(); x++)
	{
		const auto &data = queue.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		if (data.where == kOpenWaiting)
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kOpenReady)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
		else if (data.where == kClosed)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data);
		}
	}
}

#endif /* NERO_h */
