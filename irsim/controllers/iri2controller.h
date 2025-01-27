#ifndef IRI2CONTROLLER_H_
#define IRI2CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri2Controller : public CController
{
public:

    CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri2Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
		/* ROBOT */
    	CEpuck* m_pcEpuck;

		/* SENSORS */
		CWheelsActuator* m_acWheels;
    	CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;  
		CRedBatterySensor* m_seRedBattery;  
  

		/* Global Variables */
		double 		m_fLeftSpeed;
		double 		m_fRightSpeed;
		double**	m_fActivationTable;
		double 		m_fTime;
		double 		fForageToWashInhibitor;
		double 		fWashToNavigateInhibitor;
		double 		fAvoidToNavigateCountInhibitor;
		int 		justStopped;

		float 		m_fOrientation; 
		dVector2 	m_vPosition;

		int 		m_nWriteToFile;

		/* Functions */

		void ExecuteBehaviors ( void );
		void Coordinator ( void );

		void ObstacleAvoidance ( unsigned int un_priority );
		void Navigate ( unsigned int un_priority );
		void NavigateGym ( unsigned int un_priority );
		void GoLoadWash ( unsigned int un_priority );
		void Forage ( unsigned int un_priority );
};

#endif
