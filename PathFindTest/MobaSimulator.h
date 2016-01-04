#include <vector>
#include <unordered_set>
#include <unordered_map>
#define TOTAL_ATTACK_CD 1.0f
#define UNIT_RADIUS	16.0f
#define NEAR_ATTACK_RANGE (UNIT_RADIUS / 2.0f)
#define FAR_ATTACK_RANGE (UNIT_RADIUS / 2.0f)
#define MOVE_SPEED 100.0f
#define HEALTH_POINT 100

namespace UnitState{
	enum UnitState{
		Attacking, Moving
	};
}

class Soldier{
public:
	float x, y;
	float goal_x, goal_y;
	float temp_goal_x, temp_goal_y;
	int team;
	int hp;
	unsigned int unit_id;
	Soldier(float x, float y, float goal_x, float goal_y, int team)
		:x(x), y(y), goal_x(goal_x), goal_y(goal_y), team(team), hp(HEALTH_POINT), attack_count(0){
	}
	float left_cd;
	float attcking_range;
	unsigned int attack_count;
	UnitState::UnitState unit_state;
public:
	void isAttacked(Soldier *source);
	void attacking(Soldier *target);

};

namespace RVO{
	class RVOSimulator;
}

class MobaSimulator{
public:
	friend class Soldier;
	friend bool searchEnemy(MobaSimulator *sim, Soldier *soldier, float delta_time);
	friend bool moveToFinalDestination(MobaSimulator *sim, Soldier *soldier, float delta_time);
	friend void cvMouseCallback(int mouseEvent, int x, int y, int flags, void* param);
	friend void getDirectionPoint(MobaSimulator *sim, Soldier *soldier, float *des_x, float *des_y);
	typedef bool(*Behavior)(MobaSimulator *, Soldier *, float delta_time);
public:
	MobaSimulator(float height, float width);
	~MobaSimulator();
	void update(float delta_time);
	void show();
private:
	Soldier* createNewSoldier(int team, int index);
	bool eraseSoldier(unsigned int unit_id);
	void setUnitMoveDestination(int unitID, float dx, float dy);
	void processAttack(Soldier *source, Soldier *target);
	void refreshSoldiers();
	void initObstacles();
	void initOpenCV();
	void initHero();
private:
	float m_height, m_width;
	unsigned long long m_timestamp;
	std::unordered_map<unsigned int, Soldier *> m_soldier_map;
	std::vector<Behavior> m_behavior_list;
	unsigned int m_total_unit;
	RVO::RVOSimulator *m_rvo_simulator;
	std::unordered_map<unsigned int, size_t>  m_unit_to_agent;
	std::vector<unsigned int> m_agent_to_unit;
	unsigned long long m_frame_count;
	unsigned int m_hero_id;
};


bool searchEnemy(MobaSimulator *sim, Soldier *soldier, float delta_time);
bool attackCD(MobaSimulator *sim, Soldier *soldier, float delta_time);