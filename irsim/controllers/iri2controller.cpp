/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "redbatterysensor.h"

/******************** Actuators ***************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri2controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

/******************** Priorities **************/

#define BEHAVIORS	5

#define AVOID_PRIORITY 0
#define FORAGE_PRIORITY 1
#define RELOAD_PRIORITY 2
#define NAVIGATE_PRIORITY_GYM 3
#define NAVIGATE_PRIORITY 4

/******************** Thresholds **************/

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.8

/* Threshold to define the battery discharged */
#define BATTERY_WASH_THRESHOLD 0.25

/* Threshold to find the gym area */
#define NAVIGATE_GYM_THRESHOLD 0.2

#define SPEED 500


/******************************************************************************/
/******************************************************************************/

CIri2Controller::CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);

	/* Initilize Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;
	justStopped = 0.0;

	/* Initialize Inhibitors */
	fForageToWashInhibitor = 1.0;
	fWashToNavigateInhibitor = 1.0;
	fAvoidToNavigateCountInhibitor = 1.0;

	/* Create TABLE for the COORDINATOR */
	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}

}

/******************************************************************************/
/******************************************************************************/

CIri2Controller::~CIri2Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	if (m_nWriteToFile ) 
	{
	/* INIT: WRITE TO FILES */
		/* Write robot position and orientation */
		FILE* filePosition = fopen("outputFiles/robotPosition", "a");
		fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
		fclose(filePosition);

		/* Write robot wheels speed */
		FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
		fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileWheels);
		/* END WRITE TO FILES */
	}
	
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fForageToWashInhibitor = 1.0;
	fWashToNavigateInhibitor = 1.0;
	fAvoidToNavigateCountInhibitor = 1.0;

	/* Set Leds to YELLOW */
	m_pcEpuck->SetAllColoredLeds( LED_COLOR_YELLOW );
	
	ObstacleAvoidance ( AVOID_PRIORITY );
	Forage ( FORAGE_PRIORITY );
  	GoLoadWash ( RELOAD_PRIORITY );
	Navigate ( NAVIGATE_PRIORITY );
	NavigateGym ( NAVIGATE_PRIORITY_GYM );
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::Coordinator ( void )
{
  /* Read Battery Sensors */
  double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);

  int nBehavior;
  double fAngle = 0.0;

  int nActiveBehaviors = 0;
  
  /* Create vector of movement */
  dVector2  vAngle;
  vAngle.x = 0.0;
  vAngle.y = 0.0;
  
  /* For every Behavior */
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
    /* If behavior is active */
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
      	/* DEBUG */
      	printf("Behavior: %d Bateria roja: %2f\n", nBehavior, redbattery[0]);
      	/* DEBUG */
      	vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
      	vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
		}
	}

  /* Calc angle of movement */
  fAngle = atan2(vAngle.y, vAngle.x);
  
  /* Normalize fAngle */
  while ( fAngle > M_PI ) fAngle -= 2 * M_PI;
  while ( fAngle < -M_PI ) fAngle += 2 * M_PI;
 
  /* Based on the angle, calc wheels movements */
  double fCLinear = 1.0;
  double fCAngular = 1.0;
  double fC1 = SPEED / M_PI;

  /* Calc Linear Speed */
  double fVLinear = SPEED * fCLinear * ( cos ( fAngle / 2) );

  /*Calc Angular Speed */
  double fVAngular = fAngle;

  m_fLeftSpeed  = fVLinear - fC1 * fVAngular;
  m_fRightSpeed = fVLinear + fC1 * fVAngular;


  printf("LEFT: %2f RIGHT: %2f\n", m_fLeftSpeed, m_fRightSpeed);
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	
	/* Read Proximity Sensors */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);

	/* Create repelent angle */
	fRepelent -= M_PI;

	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  	m_fActivationTable[un_priority][0] = fRepelent;
  	m_fActivationTable[un_priority][1] = fMaxProx;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		if ( justStopped < 100 )
		{
			justStopped += 1;
		}

		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds( LED_COLOR_GREEN );

    	/* Mark Behavior as active */
    	m_fActivationTable[un_priority][2] = 1.0;

		/* Inibit Navigate */
		fAvoidToNavigateCountInhibitor = 0.0;

	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::Navigate ( unsigned int un_priority )
{

	/* Read BlueLight Sensors */
  	double* blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	double fTotalBlueLight = 0.0;
	double fMaxBlueLight = 0.0;
	const double* bluelightDirections = m_seBlueLight->GetSensorDirections();

	/* Read Ground Memory Sensors */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);

	/* Read Ground Sensors */
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);

	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += blueLight[i] * cos ( bluelightDirections[i] );
		vRepelent.y += blueLight[i] * sin ( bluelightDirections[i] );

		if ( blueLight[i] > fMaxBlueLight )
			fMaxBlueLight = blueLight[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++)
	{
		fTotalBlueLight += blueLight[i];
	}

	/* DEBUG */
	printf("GroundMemorySensor: %2f\n", groundMemory[0]);
	printf("GroundSensor: %2f\n", ground[0]);
	printf("JustStopped: %2i\n", justStopped);
	/* DEBUG */
	
	/* If not in black area and not pointing to the light */
	if ( ground[0] == 1.0 && groundMemory[0] == 0.0  && justStopped == 0.0 && (blueLight[0] * blueLight[7]) == 0.0 && fWashToNavigateInhibitor)
	{
		/* Set Leds to WHITE */
		m_pcEpuck->SetAllColoredLeds( LED_COLOR_WHITE );

		m_fActivationTable[un_priority][0] = fRepelent;
  		m_fActivationTable[un_priority][1] = fMaxBlueLight;
	}
	else
	{
		if ( ( justStopped > 0 ) && fAvoidToNavigateCountInhibitor == 1.0 ) 
		{
			justStopped = justStopped - 1.0;
		}

		m_fActivationTable[un_priority][0] = 0.0;
		m_fActivationTable[un_priority][1] = 0.5;
	}

	/* Mark behavior as active */
	m_fActivationTable[un_priority][2] = 1.0;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */ 
	}
}
		
/******************************************************************************/
/******************************************************************************/

void CIri2Controller::NavigateGym ( unsigned int un_priority )
{
	/* Read Light Sensors */
  	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	double fMaxLight = 0.0;
	double fTotalLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;


  	m_fActivationTable[un_priority][0] = fRepelent;
  	m_fActivationTable[un_priority][1] = fMaxLight;

	for ( int i = 0; i < m_seLight->GetNumberOfInputs() ; i ++)
	{
		fTotalLight += light[i];
	}

	/* If above a threshold */
	if ( fTotalLight * fWashToNavigateInhibitor > NAVIGATE_GYM_THRESHOLD )
	{
		/* Mark behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
		
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::GoLoadWash ( unsigned int un_priority )
{
	/* Read RedBattery Sensors */
	double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);

	/* Read RedLight Sensors */
	double* redlight = m_seRedLight->GetSensorReading(m_pcEpuck);

	double fMaxRedLight = 0.0;
	const double* redlightDirections = m_seRedLight->GetSensorDirections();

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += redlight[i] * cos ( redlightDirections[i] );
		vRepelent.y += redlight[i] * sin ( redlightDirections[i] );

		if ( redlight[i] > fMaxRedLight )
			fMaxRedLight = redlight[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;


  	m_fActivationTable[un_priority][0] = fRepelent;
  	m_fActivationTable[un_priority][1] = fMaxRedLight;

	/* If battery below a BATTERY_WASH_THRESHOLD */
	if ( ( redbattery[0] < BATTERY_WASH_THRESHOLD ) && fForageToWashInhibitor == 1.0 ) 
	{
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
		
    	/* Mark behavior as active */
    	m_fActivationTable[un_priority][2] = 1.0;

		/* Inibit Navigate */
		fWashToNavigateInhibitor = 0.0;

	}	

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, redbattery[0], redlight[0], redlight[1], redlight[2], redlight[3], redlight[4], redlight[5], redlight[6], redlight[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri2Controller::Forage ( unsigned int un_priority )
{
	/* Read Ground Memory Sensors */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Read BlueLight Sensors */
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	
	double fMaxBlueLight = 0.0;
	const double* bluelightDirections = m_seBlueLight->GetSensorDirections();

  	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += bluelight[i] * cos ( bluelightDirections[i] );
		vRepelent.y += bluelight[i] * sin ( bluelightDirections[i] );

		if ( bluelight[i] > fMaxBlueLight )
			fMaxBlueLight = bluelight[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);

	/* Create repelent angle */
	fRepelent -= M_PI;
	
  	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 1 - fMaxBlueLight;
  
  	/* If with a virtual puck */
	if ( groundMemory[0] == 1.0 && (justStopped == 0))
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);

    	/* Mark Behavior as active */
    	m_fActivationTable[un_priority][2] = 1.0;

		/* Inibit GoLoadWash */
		fForageToWashInhibitor = 0.0;
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/forageOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, groundMemory[0], bluelight[0], bluelight[1], bluelight[2], bluelight[3], bluelight[4], bluelight[5], bluelight[6], bluelight[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}

}

