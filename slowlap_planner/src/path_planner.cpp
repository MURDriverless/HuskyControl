/**
 * This handles the path planning algorithm
 * node.cpp passes cone and car info through the function update(...)
 * - add new cones 
 * - sort cones by colour, and by correct order in race track (not necessarily by distance)
 * - generate path points by taking the mid point between 2 opposite cones 
 * then passes back path information to node.cpp
 * 
 * for future improvement: generate velocity reference as well
 * 
 * author: Aldrei (MURauto21)
 **/

#include "path_planner.h"

//constructor
PathPlanner::PathPlanner(float car_x, float car_y, std::vector<Cone> &cones, bool const_velocity, float v_max, float v_const, float max_f_gain, std::vector<PathPoint>&markers)
    : const_velocity(const_velocity), v_max(v_max), v_const(v_const), f_gain(max_f_gain), car_pos(PathPoint(car_x,car_y))
{
	//set capacity of vectors
	raw_cones.reserve(500);
	l_cones_to_add.reserve(50);
	r_cones_to_add.reserve(50);
	left_cones.reserve(250);
	right_cones.reserve(250);
	addedIDLeft.reserve(250);
	addedIDRight.reserve(250);
	addedIDRed.reserve(250);
	centre_points.reserve(300);
	rejected_points.reserve(300);
	thisSide_cone.reserve(150);
	oppSide_cone.reserve(150);

	///problem: if SLAM doesnt give leass than 4 cones at first
	addCones(cones);								// add new cones to raw cones
    centre_points.emplace_back(car_x,car_y);		// add the car's initial position to centre points  
	centralizeTimingCones();						// add orange cones midpoint to centre points
	sortConesByDist(car_pos);						// sort first seen cones by distance
	if (timingCalc)
		addCentrePoints();
	if (DEBUG) std::cout << "path points size (initial): " << centre_points.size() <<std::endl; //this should give 3 under normal circumstances
	resetTempConeVectors();							// Clear pointers and reset l/r_cones_to_add
}

// takes car and cone infor from node.cpp then update pathpoints to be passed back to node.cpp
void PathPlanner::update(std::vector<Cone> &new_cones, const float car_x, const float car_y,
						 std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V,
						 std::vector<Cone> &Left,std::vector<Cone> &Right, std::vector<PathPoint> &markers, bool&plannerComp)
{
	if (complete) // if race track is complete
	{	returnResult(X, Y, V,Left,Right,markers);
		plannerComp = true;
	}
	else
	{
		car_pos = PathPoint(car_x,car_y); //update car's position
		
		if (left_start_zone)
		{
			// join track if feasible
			if (joinFeasible(car_x, car_y))
			{
				ROS_INFO_STREAM("[PLANNER] Race track almost complete");
				centre_points.push_back(centre_points.front());
				reached_end_zone = true;
				complete = true;

			}
		}
		else
		{
			if (calcDist(centre_points.front(), car_pos) > 5) //if greater than 5m away
				left_start_zone = true;
		}
		
		if (!reached_end_zone)
		{
			addCones(new_cones);
			updateCentrePoints();
			if (!timingCalc) // if failed the first time
			{
				centralizeTimingCones();
			}

			if (timingCalc && newConesToSort)
			{
				if (left_cones.empty() || right_cones.empty())
				{
					sortConesByDist(car_pos);
				}
				if (!left_cones.empty() && !right_cones.empty())
				{
					sortAndPushCone(l_cones_to_add);
					sortAndPushCone(r_cones_to_add);			
					addCentrePoints();
				}
			}
		}

		returnResult(X, Y, V, Left, Right,markers);	
		
		resetTempConeVectors();
	}
}

// checks whether track is (almost) finished
bool PathPlanner::joinFeasible(const float &car_x, const float &car_y)
{
	float dist = calcDist(centre_points.back(), centre_points.front());
	if (DEBUG) std::cout<<"[PLANNER] Distance of latest path point to finish line: "<<dist<<std::endl;
	if ( dist < 2) //if less than 2 meters 
	{
		float angle = calcAngle(*(centre_points.end() - 2), centre_points.back(), centre_points.front());
		std::cout << angle << std::endl;
		if (abs(angle) < MAX_PATH_ANGLE1 || abs(angle)> MAX_PATH_ANGLE2)
		{
			return true;
		}
	}
	else
		return false;
}

// sets the vectors to be returned to node.cpp
void PathPlanner::returnResult(std::vector<float> &X, std::vector<float> &Y, std::vector<float> &V,
								std::vector<Cone>&Left, std::vector<Cone>&Right,std::vector<PathPoint>&markers)
{
	for (auto &e: centre_points)
	{
		X.push_back(e.x); 
		Y.push_back(e.y); 
		V.push_back(e.velocity);
		if (e.cone1 != NULL)
		{
			markers.push_back(e.cone1->position);
			markers.back().accepted = true;
			markers.push_back(e.cone2->position);
			markers.back().accepted = true;
		}
	}

	if (!rejected_points.empty())
	{
		count++;
		for (auto &r: rejected_points)
		{
			markers.push_back(r.cone1->position);
			markers.back().accepted = false;
			markers.push_back(r.cone2->position);
			markers.back().accepted = false;
		}
	}

	for (auto lc:left_cones)
	{
		Left.push_back(*lc);
	}

	for (auto rc:right_cones)
	{
		Right.push_back(*rc);
	}
}


// not used? (MURauto20)
void PathPlanner::shutdown()
{
    left_cones.clear();
    right_cones.clear();
    timing_cones.clear();
    l_cones_to_add.clear();
    r_cones_to_add.clear();
}

// calculates the angle difference. used for cone sorting
float PathPlanner::calcAngle(const PathPoint &A, const PathPoint &B, const PathPoint &C)
{
	float cb_x = C.x - B.x;
	float cb_y = C.y - B.y;
	float ca_x = B.x - A.x;
	float ca_y = B.y - A.y;

	float angle = calcRelativeAngle(C,B) - calcRelativeAngle(B,A);

	return angle;
}

// calculates angle between 2 points (global frame)
float PathPlanner::calcRelativeAngle(const PathPoint &p1, const PathPoint &p2)
{
	float angle = (atan2(p2.y - p1.y, p2.x - p1.x))* 180 / M_PI;
	return angle;
}

// generates path points by getting the mid point between 2 cones.
// points can be accepted or rejected, see if/else conditions 
PathPoint PathPlanner::generateCentrePoint(Cone* cone_one, Cone* cone_two, bool& feasible)
{
	PathPoint midpoint(
		(cone_one->position.x + cone_two->position.x) / 2,
		(cone_one->position.y + cone_two->position.y) / 2
	);

	// calc the distance to the latest path point
	float dist_back = calcDist(centre_points.back(), midpoint);
	//calc the angle difference
	float angle1 = calcRelativeAngle(centre_points.back(),midpoint); 
	float angle2 = calcRelativeAngle(*(centre_points.end()-2),centre_points.back());
	float angle = angle1 - angle2; //same as calcAngle(...)
	
	
	if ((abs(angle)<MAX_PATH_ANGLE1 ||abs(angle)>MAX_PATH_ANGLE2) && (dist_back>MIN_POINT_DIST) && (dist_back<MAX_POINT_DIST))
	{
		// if (DEBUG) std::cout << "Accepted path point: (" << midpoint.x << ", " << midpoint.y << ") ";
		// if (DEBUG) std::cout << "dist and angle: " << dist_back << " " << angle << std::endl;
		feasible = true;
		midpoint.cone1 = cone_one;
		midpoint.cone2 = cone_two;
	}
	else
	{
		if (DEBUG) std::cout << "[XX] Rejected point: (" << midpoint.x << ", " << midpoint.y << ") ";
		// std::cout<<"cone one: ("<<cone_one->position.x<<", "<<cone_one->position.y<<") ";
		// std::cout<<"cone two: ("<<cone_two->position.x<<", "<<cone_two->position.y<<")   ";
		if (DEBUG) std::cout<<"previous points: ("<<centre_points.back().x<<", "<<centre_points.back().y<<")";
		if (DEBUG) std::cout<<" ("<<(*(centre_points.end()-2)).x<<", "<<(*(centre_points.end()-2)).y<<")";
		if (DEBUG) std::cout << " dist and angle: " << dist_back << " " << angle1<<" - "<<angle2 <<std::endl;
		feasible = false;
		midpoint.cone1 = cone_one;
		midpoint.cone2 = cone_two;
		rejected_points.push_back(midpoint);
	}

	return midpoint;
}

// adds new path points to vector centre_points. uses generateCentrePoint()
void PathPlanner::addCentrePoints()
{
	if (left_cones.empty() || right_cones.empty())
		return;

	bool feasible;
	PathPoint cp;
	int indx;
	Cone* opp_cone;
	thisSide_cone.clear();
	oppSide_cone.clear();


	//we will look at the side with more cones seen
	if (right_cones.size() > left_cones.size())
	{
		for (auto &con:right_cones)
			thisSide_cone.push_back(con);
		for (auto &con:left_cones)
			oppSide_cone.push_back(con);
		indx = rightIndx;
	}
	else if (left_cones.size() >= right_cones.size())
	{
		for (auto &con:left_cones)
			thisSide_cone.push_back(con);
		for (auto &con: right_cones)
			oppSide_cone.push_back(con);
		indx = leftIndx;
	}
	
	for (int i = indx; i < thisSide_cone.size(); i++)
	{
		if((!thisSide_cone[i]->passedBy)||(thisSide_cone[i]->paired<3))
		{
			opp_cone = findOppositeClosest(*thisSide_cone[i], oppSide_cone);
			feasible = false;
			cp = generateCentrePoint(thisSide_cone[i], opp_cone, feasible);
			if (feasible)
			{
				centre_points.push_back(cp);
				cp.cone1->paired ++;
				cp.cone2->paired ++;	
				thisSide_cone[i]->mapped++;
				opp_cone->mapped++;
			}
		}
	}



}

//add new cones to the local vector of cones and sort by colour
void PathPlanner::addCones(std::vector<Cone> &new_cones)
{
	updateStoredCones(new_cones);
	int temp = left_cones.size() + right_cones.size() + timing_cones.size();
	if (DEBUG)
	{
		std::cout<<"\nSLAM gives  "<<new_cones.size()<<" cones.";
		std::cout<<" left saved cones: "<<left_cones.size();
		std::cout<<". right saved cones: "<<right_cones.size();
		std::cout<<". timing cones: "<<timing_cones.size();
		std::cout<<". future cones: "<<future_cones.size()<<std::endl;
	}
	if (gotNewCones || !passedByAll)
	{

		for (auto &cone: future_cones)
		{
			if (cone->colour == 'b')
			{
				l_cones_to_add.push_back(cone);
				l_cones_sorted = false;
				newConesToSort = true;
			}

			else if (cone->colour == 'y')
			{
				r_cones_to_add.push_back(cone);
				r_cones_sorted = false;
				newConesToSort = true;
			}
			else
			{
				if (timing_cones.size()<2)
				{
					timing_cones.push_back(cone);
					addedIDRed.push_back(timing_cones.back()->id);
					// timing_cones.back()->passedBy = true; //set to true right away for timing cones
					if (DEBUG) std::cout<<"Timing cones found: "<< timing_cones.size() <<std::endl;	
				}
			}
		}
		if (DEBUG)	std::cout<<"l and r future cones: "<<l_cones_to_add.size()<<" and "<<r_cones_to_add.size()<<std::endl;
		
	}

	else
		return;	
}

// update the position of the raw cones using new cone pos
void PathPlanner::updateStoredCones(std::vector<Cone>&new_cones)
{

	float dist;
	//add new cones to raw cones while updating future cones
	for (int i=0; i<new_cones.size();i++)
	{
		if (i==raw_cones.size()) //add newly seen cones
		{
			raw_cones.push_back(new_cones[i]);
			future_cones.push_back(&raw_cones[i]);
		}

		else //update previously seen cones if not yet passed by
		{			
			if (!raw_cones[i].passedBy)
			{
				dist = calcDist(raw_cones[i].position,car_pos); //check if within range
				if(dist<CERTAIN_RANGE)
				{
					raw_cones[i].passedBy = true;
				}
				else //if not yet within range
				{
					raw_cones[i].updateConePos(new_cones[i].position);
					future_cones.push_back(&raw_cones[i]);
				}
			}
		}
	}

	if (raw_cones.size()!=new_cones.size())
		gotNewCones = true;

	if (passedByIndex == raw_cones.size()-1)
	{
		passedByAll = true;
	}
	 
	//update left and right cones
	if (left_cones.size()>0)
	{
		for(int i = left_cones.size()-1;i>=0; i--)
		{
			if (!left_cones[i]->passedBy)
				{
					left_cones.pop_back();
				}
			else
				{
					leftIndx = i;
					break;
				}
				
		}
	}

	if (right_cones.size()>0)
	{
		for(int i = right_cones.size()-1; i>=0; i--)
		{
			if (!right_cones[i]->passedBy)
				{
					right_cones.pop_back();
				}
			else
				{
					rightIndx = i;
					break;
				}
		}
	
	}
	
}


void PathPlanner::updateCentrePoints()
{
	
	int temp = centre_points.size();
	float dist;
	float min_dist = 9000;
	int nearest_indx=-1;
	std::cout<<"centre points size before update: "<<temp;
	if (temp <= 2)
	{	
		std::cout<<"\n";
		return;
	}
	else
	{
		//searchfor the nearest path point
		for (int i = centre_points.size()-1;i>=0;i--)
		{
			dist = calcDist(car_pos,centre_points[i]);
			if (dist<min_dist)
			{
				min_dist = dist;
				nearest_indx = i;
			}
			else
				break;
		}
		
		// pop path points if their cones havent been passed by yet
		for (int i = centre_points.size()-1;i>=0;i--)
		{
			if(!centre_points[i].cone1->passedBy)
			{
				centre_points.back().cone1->paired --;
				centre_points.back().cone2->paired --;
				centre_points.pop_back();
			}
			else
				break;
		}
		 
		//experimental: i want to only have at least 2 path points ahead, so pop
		if (nearest_indx!=-1 || (nearest_indx+2) < centre_points.size()-1)
		{
			for (int i = centre_points.size()-1;i>nearest_indx+2;i--)
			{
				centre_points.back().cone1->paired --;
				centre_points.back().cone2->paired --;
				centre_points.pop_back();
			}
		}


		std::cout<<"  centre points size after update: "<<centre_points.size()<<std::endl;
	}
}
     

// get midpoint of orange cones
void PathPlanner::centralizeTimingCones()
{
	/*taking the average ditance of the timing cones is same as getting their midpoint */
	
	PathPoint avg_point(0, 0);
	
	for (int i = 0; i < timing_cones.size(); i++)
	{
		avg_point.x += timing_cones[i]->position.x; // summation of x positions
		avg_point.y += timing_cones[i]->position.y; // summation of y positions
	} 
	avg_point.x = avg_point.x / timing_cones.size(); // avg x dist
	avg_point.y = avg_point.y / timing_cones.size(); // avg y dist


	// Calc distance to timing cone
	PathPoint coneTemp(timing_cones.front()->position.x,timing_cones.front()->position.y);
	float dist = calcDist(coneTemp, avg_point);
	float angle = calcRelativeAngle(centre_points.back(), avg_point);
	
	//this will potentially give an error especially if car's initial pose is very close to this point
	// if (dist > 0.1*TRACKWIDTH && dist < TRACKWIDTH && (abs(angle)<10))
	if  (abs(angle)<10)
	{
		centre_points.push_back(avg_point);
		if (DEBUG) std::cout << "Average timing cones position calculated" <<std::endl;
		std::cout << "Accepted point: (" << avg_point.x << ", " << avg_point.y<<")"<<std::endl;
		timingCalc = true;
		std::cout << "path points size (timing): " << centre_points.size() <<std::endl;
		centre_points.back().cone1 = timing_cones.front();
		centre_points.back().cone2 = timing_cones.back();

	}
	else
	{
		if (DEBUG) std::cout << "[XX] Average timing cones position NOT calculated" <<std::endl;
		std::cout << "Rejected point: (" << avg_point.x << ", " << avg_point.y<<")"<<std::endl;
		if (DEBUG) std::cout << "dist and angle: " << dist << " " << angle <<std::endl;
		timingCalc = false;
	}

		
	
}

// find the closest cone on the opposite side
//this is mostly used on the last cone in the vector,
Cone* PathPlanner::findOppositeClosest(const Cone &cone, const std::vector<Cone*> &cones)
{
	float min_dist = 9999; //large number
	float dist;
	Cone* closest_cone = cones.back();
	int count = 0;
	for (int i = cones.size()-1; i>=0;i--)
	{
		dist = calcDist(cone.position, cones[i]->position);
		if (dist < min_dist)
		{
			// count = 0;
			min_dist = dist;
			closest_cone = cones[i];
		}
		// if after 5 cones, the closest cone has not been replace, break and return current closest cone
		// this keeps it from going through all the cones
		// if (count == 5) break; 
		// count ++;
	}
	return closest_cone;
}

//sorts the cones by distance to car, then adds the closes cone to left/right
void PathPlanner::sortConesByDist(const PathPoint &pos)
{
	if (l_cones_to_add.empty() || r_cones_to_add.empty())
		return;

	// Assign distance Cone objects on left
    for (auto &cone: l_cones_to_add)
    {cone->dist = calcDist(pos, cone->position);}

	// Assign distance to Cone objects on right
    for (auto &cone: r_cones_to_add) 
    {cone->dist = calcDist(pos, cone->position);}

	// nlogn sort both cones_to_add vectors
	if (l_cones_to_add.size()>1)
    sort(l_cones_to_add.begin(), l_cones_to_add.end(), compareConeDist);
	if (r_cones_to_add.size()>1)
    sort(r_cones_to_add.begin(), r_cones_to_add.end(), compareConeDist);

    l_cones_sorted = true;
    r_cones_sorted = true;

	// add only the nearest left and right cone
	left_cones.push_back(l_cones_to_add.front());	
	right_cones.push_back(r_cones_to_add.front());
	addedIDLeft.push_back(left_cones.back()->id);
	addedIDRight.push_back(right_cones.back()->id);
	removeFirstPtr(l_cones_to_add);
	removeFirstPtr(r_cones_to_add);
}
bool PathPlanner::compareConeDist(Cone* const &cone_one, Cone* const &cone_two)
{
    return cone_one->dist < cone_two->dist;
}
bool PathPlanner::compareConeCost(Cone* const &cone_one, Cone* const &cone_two)
{
    return cone_one->cost < cone_two->cost;
}

/* Calculate the distance between 2 points */
float PathPlanner::calcDist(const PathPoint &p1, const PathPoint &p2)
{
    float x_dist = pow(p2.x - p1.x, 2);
    float y_dist = pow(p2.y - p1.y, 2);

    return sqrt(x_dist + y_dist);
}

void PathPlanner::resetTempConeVectors()
{
	l_cones_to_add.clear();
	r_cones_to_add.clear();
	oppSide_cone.clear();
	thisSide_cone.clear();
	future_cones.clear();
	l_cones_sorted = false;
	r_cones_sorted = false;
	newConesToSort = false;
	newConesSorted = false;
	gotNewCones = false;
	if (count > 5)
	rejected_points.clear();
}

void PathPlanner::removeFirstPtr(std::vector<Cone*>& cone_vec)
{
    if (cone_vec.size() > 0 && cone_vec.front() != NULL)
    {
        cone_vec.erase(cone_vec.begin());
    }
}

// cost 1: distance between cones of same colour
float PathPlanner::computeCost1(Cone* &cn1, Cone* &cn2)
{
	return calcDist(cn1->position,cn2->position);
}

// cost 2: distance between cone from opposite side
float PathPlanner::computeCost2(Cone* &cn1, std::vector<Cone*> &oppCone1,std::vector<Cone*> &oppCone2)
{
	Cone* opp_cone = findOppositeClosest(*cn1,oppCone1);
	float dist1 = calcDist(cn1->position,opp_cone->position);
	float dist2 = 99; //random large number
	if (oppCone2.size()>0)
	{
		Cone* opp_cone2 = findOppositeClosest(*cn1,oppCone2);
		dist2 = calcDist(cn1->position,opp_cone2->position);
	}

	return std::min(dist1,dist2);
}

// cost 3: change in track curvature cn2 is sorted cone
float PathPlanner::computeCost3(Cone* &cn1, std::vector<Cone*> &cn2)
{
	if (cn2.size()<2)
		return 0;
	int i = cn2.size()-1;
	float theta1 =  atan2((cn2[i]->position.y - cn2[i-1]->position.y),(cn2[i]->position.x - cn2[i-1]->position.x));
	float theta2 =  atan2((cn2[i]->position.y - cn1->position.y),(cn2[i]->position.x - cn1->position.x));
	return theta1 - theta2;
}

//lr-cones_to_add are the newly seen cones
//leftright cones are the sorted
//from the beginning
void PathPlanner::sortAndPushCone(std::vector<Cone*> &cn)
{
	// newConesSorted = false;
	//if empty, return
	if (cn.size() == 0)
		return;

	float cost, cost1, cost2, cost3;
	int index = 0;
	char colour;
	thisSide_cone.clear();
	oppSide_cone.clear();
	oppSide_cone2.clear();

	if (cn.front()->colour == 'b') //if cone is blue(left)
	{
		colour = 'b';
		thisSide_cone.assign(left_cones.begin(),left_cones.end());
		oppSide_cone.assign(right_cones.begin(),right_cones.end());
		if (r_cones_to_add.size()>0)
			oppSide_cone2.assign(r_cones_to_add.begin(),r_cones_to_add.end());

	}
	else if (cn.front()->colour == 'y') //if cone is yellow(right)
	{
		colour = 'y';
		thisSide_cone.assign(right_cones.begin(),right_cones.end());
		oppSide_cone.assign(left_cones.begin(),left_cones.end());
		if (l_cones_to_add.size()>0)
			oppSide_cone2.assign(l_cones_to_add.begin(),l_cones_to_add.end());
	}
	
	std::cout<<"cn size: "<<cn.size()<<std::endl;

	if (cn.size()<2) //if only 1 cone is seen, compute cost 2 (dist to opposite side) and compare to track width
	{
		float cost2 = computeCost2(cn.back(),oppSide_cone,oppSide_cone2);
		if (cost2 < TRACKWIDTH*1.25)
		{
			if (colour == 'b')
			{
				left_cones.push_back(cn.back());
				addedIDLeft.push_back(left_cones.back()->id);
			}
			else if (colour == 'y')
			{
				right_cones.push_back(cn.back());
				addedIDRight.push_back(right_cones.back()->id);
			}
			newConesSorted = true;
			return;
		}
		else
		{
			return;
		}
			

	}
		

	else if (cn.size() > 1)
	{
		for (int i=0;i<cn.size();i++)
		{		
			cost1 = computeCost1(cn[i],thisSide_cone.back());
			cost2 = computeCost2(cn[i],oppSide_cone,oppSide_cone2);
			cost3 = computeCost3(cn[i],thisSide_cone);
			cn[i]->cost = cost1 + cost2 + cost3;
		}

		sort(cn.begin(), cn.end(), compareConeCost);

		if (colour == 'b')
		{
			for (auto &c:cn)
			{
				left_cones.push_back(c);
				addedIDLeft.push_back(left_cones.back()->id);
			}
			
			
		}
		else if (colour == 'y')
		{
			for (auto &c:cn)
			{
				right_cones.push_back(c);
				addedIDRight.push_back(right_cones.back()->id);
			}
		}
		newConesSorted = true;
	}

	

}


