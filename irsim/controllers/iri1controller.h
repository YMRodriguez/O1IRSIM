#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:
	CIri1Controller(const char *pch_name, CEpuck *pc_epuck, int n_wrtie_to_file);
	~CIri1Controller();
	void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
	/* ROBOT */
	CEpuck *m_pcEpuck;

	/* SENSORS */
	CWheelsActuator *m_acWheels;
	CEpuckProximitySensor *m_seProx;
	CLightSensor *m_seLight;
	CRealRedLightSensor *m_seRedLight;
	CRealBlueLightSensor *m_seBlueLight;
	CContactSensor *m_seContact;
	CGroundSensor *m_seGround;
	CGroundMemorySensor *m_seGroundMemory;
	CRedBatterySensor *m_seRedBattery;

	/* Global Variables */
	double m_fLeftSpeed;
	double m_fRightSpeed;
	double **m_fActivationTable;
	int m_nWriteToFile;
	double m_fTime;
	double fForageToWashInhibitor;
	double fWashToNavigateInhibitor;
	double fAvoidToNavigateCountInhibitor;
	int justStopped;

	/* Functions */

	void ExecuteBehaviors(void);
	void Coordinator(void);

	void ObstacleAvoidance(unsigned int un_priority);
	void Forage(unsigned int un_priority);
	void GoLoadWash(unsigned int un_priority);
	void Navigate(unsigned int un_priority);
	void NavigateGym(unsigned int un_priority);
};

#endif
