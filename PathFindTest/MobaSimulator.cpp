#include <stdio.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "MobaSimulator.h"
#include "RVOSimulator.h"

MobaSimulator::MobaSimulator(float height, float width) :
m_height(height), m_width(width), m_timestamp(0ULL), m_total_unit(0), m_frame_count(0ULL)
{
	m_behavior_list.push_back(attackCD);
	m_behavior_list.push_back(searchEnemy);
	m_behavior_list.push_back(moveToFinalDestination);
	m_rvo_simulator = new RVO::RVOSimulator(0.099f, 100.0f, 100, 1.0f, 1.0f, 30.0f, 0.0f);
	initObstacles();
	initOpenCV();
	initHero();
}

Soldier* MobaSimulator::createNewSoldier(int team, int index)
{
	Soldier *result;
	float startx, starty;
	if (team == 0){
		result = new Soldier((float)index * UNIT_RADIUS * 2 + UNIT_RADIUS, (float)index * UNIT_RADIUS * 2 + UNIT_RADIUS, m_width, m_height, team);
	}
	else {
		result = new Soldier(m_width - (float)index * UNIT_RADIUS * 2 - UNIT_RADIUS, m_height - (float)index * UNIT_RADIUS * 2 - UNIT_RADIUS, 0.f, 0.f, team);
	}
	if (index >= 3){
		result->attcking_range = NEAR_ATTACK_RANGE; 
	}
	else {
		result->attcking_range = FAR_ATTACK_RANGE;
	}
	result->unit_id = m_total_unit++;
	result->left_cd = TOTAL_ATTACK_CD;

	size_t agent_number = m_rvo_simulator->addAgent(RVO::Vector2(result->x, result->y));
	m_rvo_simulator->setAgentRadius(agent_number, UNIT_RADIUS);
	m_agent_to_unit.push_back(result->unit_id);
	m_unit_to_agent[result->unit_id] = agent_number;
	m_soldier_map[result->unit_id] = result;
	return result;
}

void MobaSimulator::refreshSoldiers()
{
	for (int i = 0; i < 6; ++i){
		for (int team = 0; team <= 1; ++team){
			Soldier *toAdd = createNewSoldier(team, i);
		}
	}
}

void MobaSimulator::update(float delta_time)
{
	unsigned long long time_in_ms = (unsigned long long)(delta_time * 1000);
	m_timestamp += time_in_ms;
	//printf("Total time%I64ums\n", m_timestamp);
	if (m_timestamp == time_in_ms || m_timestamp / 10000 != (m_timestamp - time_in_ms) / 10000) refreshSoldiers();
	for (auto soldier_pair : m_soldier_map){
		if (soldier_pair.first == m_hero_id){
			moveToFinalDestination(this, soldier_pair.second, delta_time);
			continue;
		}
		for (auto behavior : m_behavior_list){
			if (behavior(this, soldier_pair.second, delta_time)) break;
		}
	}
	
	//处理死亡的单位
	for (std::unordered_map<unsigned int, Soldier *>::iterator it = m_soldier_map.begin(); it != m_soldier_map.end(); ){
		std::unordered_map<unsigned int, Soldier *>::iterator nxt = it;
		++nxt;
		if (it->second->hp <= 0 && it->second->unit_id != m_hero_id){
			eraseSoldier(it->first);
		}
		it = nxt;
	}
	//更新坐标
	for (auto soldier_pair : m_soldier_map){
		size_t agent_num = m_unit_to_agent[soldier_pair.first];
		m_rvo_simulator->setAgentPosition(agent_num, RVO::Vector2(soldier_pair.second->x, soldier_pair.second->y));
	}
	if (++m_frame_count % 3 == 1) m_rvo_simulator->doStep();
	for (auto soldier_pair : m_soldier_map){
		size_t agent_num = m_unit_to_agent[soldier_pair.first];
		RVO::Vector2 v = m_rvo_simulator->getAgentVelocity(agent_num);
		v = v * delta_time;
		
		soldier_pair.second->x += v.x();
		soldier_pair.second->y += v.y();
	}
	
}

bool MobaSimulator::eraseSoldier(unsigned int unit_id)
{
	std::unordered_map<unsigned int, Soldier *>::iterator it = m_soldier_map.find(unit_id);
	if (it != m_soldier_map.end()){
		delete it->second;
		m_soldier_map.erase(it);
		
		size_t ori_index = m_unit_to_agent[unit_id];
		size_t replace_agent_number = m_rvo_simulator->eraseAgent(ori_index);
		if (replace_agent_number > 0){
			unsigned int replace_unit_id = m_agent_to_unit[replace_agent_number];
			m_agent_to_unit[ori_index] = replace_unit_id;
			m_unit_to_agent[replace_unit_id] = ori_index;
		}
		m_agent_to_unit.pop_back();
		m_unit_to_agent.erase(unit_id);
		return true;
	}
	return false;
}

void MobaSimulator::setUnitMoveDestination(int unitID, float dx, float dy)
{
	size_t agent_num = m_unit_to_agent[unitID];
	Soldier *soldier = m_soldier_map[unitID];
	soldier->temp_goal_x = dx;
	soldier->temp_goal_y = dy;
	dx -= soldier->x;
	dy -= soldier->y;
	RVO::Vector2 v(dx, dy);
	if (RVO::abs(v) < 0.1) v = RVO::Vector2(0.0f, 0.0f); else v = RVO::normalize(v) * MOVE_SPEED;
	m_rvo_simulator->setAgentPrefVelocity(agent_num, v);
	//m_rvo_simulator->setAgentVelocity(agent_num, RVO::Vector2());
	m_rvo_simulator->setAgentMaxSpeed(agent_num, MOVE_SPEED);

	soldier->unit_state = UnitState::Moving;
}

MobaSimulator::~MobaSimulator()
{
	for (auto pair : m_soldier_map){
		delete pair.second;
	}
	delete m_rvo_simulator;
}

static float distance(float x1, float y1, float x2, float y2){
	x1 -= x2;
	y1 -= y2;
	return sqrtf(x1 * x1 + y1 * y1);
}

static bool canAttack(Soldier *source, Soldier *target);

bool searchEnemy(MobaSimulator *sim, Soldier *soldier, float delta_time)
{
	Soldier *bestUnit = NULL;
	float minDis = 300.f;
	for (auto other_soldier_pair : sim->m_soldier_map){
		Soldier *other_soldier = other_soldier_pair.second;
		if (other_soldier->team == soldier->team || other_soldier->unit_id == sim->m_hero_id) continue;
		float dis = distance(other_soldier->x, other_soldier->y, soldier->x, soldier->y);
		if (dis >= minDis)continue;
		minDis = dis;
		bestUnit = other_soldier;
	}
	if (bestUnit){
		if (canAttack(soldier, bestUnit)){
			sim->processAttack(soldier, bestUnit);
		}
		else {
			sim->setUnitMoveDestination(soldier->unit_id, bestUnit->x, bestUnit->y);
		}
		return true;
	}
	return false;
}

bool moveToFinalDestination(MobaSimulator *sim, Soldier *soldier, float delta_time)
{
	/*if (fabsf(soldier->goal_x - soldier->x) < 10 && fabsf(soldier->goal_y - soldier->y) < 10){
		soldier->goal_x = soldier->x;
		soldier->goal_y = soldier->y;
	}*/

	sim->setUnitMoveDestination(soldier->unit_id, soldier->goal_x, soldier->goal_y);
	return true;
}

bool attackCD(MobaSimulator *sim, Soldier *soldier, float delta_time)
{
	soldier->left_cd -= delta_time;
	if (soldier->left_cd <= 0.0f){
		return false;
	}
	return true;
}

static bool canAttack(Soldier *source, Soldier *target){
	float dis = distance(source->x, source->y, target->x, target->y);
	if (dis < source->attcking_range + UNIT_RADIUS * 2) return true;
	return false;
}

void MobaSimulator::processAttack(Soldier *source, Soldier *target){
	--target->hp;
	source->left_cd = TOTAL_ATTACK_CD;

	size_t agent_num = m_unit_to_agent[source->unit_id];
	//m_rvo_simulator->setAgentMaxSpeed(agent_num, 0.0f);
	m_rvo_simulator->setAgentPrefVelocity(agent_num, RVO::Vector2(0.0f, 0.0f));
	source->temp_goal_x = source->x;
	source->temp_goal_y = source->y;
	source->unit_state = UnitState::Attacking;
	++source->attack_count;
}


using namespace cv;

void getDirectionPoint(MobaSimulator *sim, Soldier *soldier, float *des_x, float *des_y){
	size_t agent_num = sim->m_unit_to_agent[soldier->unit_id];
	RVO::Vector2 v = sim->m_rvo_simulator->getAgentVelocity(agent_num);
	//float dx = soldier->temp_goal_x - soldier->x;
	//float dy = soldier->temp_goal_y - soldier->y;
	float dx = v.x();
	float dy = v.y();
	float len = sqrtf(dx * dx + dy * dy);
	if (len < 1e-3) {
		*des_x = soldier->x;
		*des_y = soldier->y;
		return;
	}
	dx *= UNIT_RADIUS / len;
	dy *= UNIT_RADIUS / len;
	*des_x = soldier->x + dx;
	*des_y = soldier->y + dy;
}

void MobaSimulator::show()
{
	IplImage *image = cvCreateImage(cvSize((int)m_width+1, (int)m_height + 1), IPL_DEPTH_8U, 3);
	cvSet(image, cvScalar(0));
	image->origin = 1;

	for (auto soldier_pair : m_soldier_map){
		int x = (int)soldier_pair.second->x;
		int y = (int)soldier_pair.second->y;
		CvPoint p;
		p.x = x; p.y = y;
		CvScalar color[2] = { cvScalar(255, 0, 0), cvScalar(0, 255, 0) };
		if (soldier_pair.second->attcking_range > NEAR_ATTACK_RANGE){
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 2; ++j)color[j].val[i] /= 2;
		}
		if (soldier_pair.first == m_hero_id){
			color[soldier_pair.second->team] = cvScalar(0, 0, 255);
		}
		char s[100];
		sprintf(s, "%d-%u", soldier_pair.second->hp, soldier_pair.second->attack_count); 
		CvFont font;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

		//cvPutText(image, s, p, &font, cvScalar(255, 255, 255));
		cvCircle(image, p, (int)UNIT_RADIUS, color[soldier_pair.second->team]);
		float bx, by;
		getDirectionPoint(this, soldier_pair.second, &bx, &by);
		cvLine(image, p, cvPoint(bx,by), cvScalar(255, 255, 255));
	}
	cvShowImage("hahaha", image);
	//printf("beforeWaitkey\n");
	waitKey(5);
	//printf("afterWaitkey\n");
	cvReleaseImage(&image);
}

void MobaSimulator::initObstacles()
{ 
	RVO::Vector2 boundary[4] = {
		RVO::Vector2(0.0f, 0.0f),
		RVO::Vector2((float)m_height, 0.0f),
		RVO::Vector2(m_height, m_width),
		RVO::Vector2(0.0f, m_width) };
	std::vector< RVO::Vector2 > segment(2);
	for (int i = 0; i < 4; ++i){
		int nxt = i + 1;
		if (nxt == 4) nxt = 0;
		segment[0] = boundary[i];
		segment[1] = boundary[nxt];
		m_rvo_simulator->addObstacle(segment);
	}
	std::vector< RVO::Vector2 > ob;
	/*ob.push_back(RVO::Vector2(400.0f, 400.0f));
	ob.push_back(RVO::Vector2(600.0f, 400.0f));
	ob.push_back(RVO::Vector2(600.0f, 600.0f));
	ob.push_back(RVO::Vector2(400.0f, 600.0f));
	m_rvo_simulator->addObstacle(ob);*/

	m_rvo_simulator->processObstacles();

}

void cvMouseCallback(int mouseEvent, int x, int y, int flags, void* param){
	MobaSimulator *sim = (MobaSimulator *)param;
	y = sim->m_height - y;
	if (mouseEvent == CV_EVENT_RBUTTONDOWN){
		Soldier *hero = sim->m_soldier_map[sim->m_hero_id];
		hero->goal_x = (float)x;
		hero->goal_y = (float)y;
	}
}

void MobaSimulator::initOpenCV()
{
	cvNamedWindow("hahaha");
	cvSetMouseCallback("hahaha", cvMouseCallback, this);
}

void MobaSimulator::initHero()
{
	Soldier *hero = createNewSoldier(0, 0);
	hero->x = m_width / 2;
	hero->y = m_height / 2;
	m_hero_id = hero->unit_id;
	hero->goal_x = hero->x;
	hero->goal_y = hero->y;
}

