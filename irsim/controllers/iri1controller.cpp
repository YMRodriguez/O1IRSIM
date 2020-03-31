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
#include "lightsensor.h"
#include "reallightsensor.h"
#include "realredlightsensor.h"
#include "realbluelightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "redbatterysensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"


/******************************************************************************/
/******************************************************************************/

extern gsl_rng* rng;
extern long int rngSeed;
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS	5

#define AVOID_PRIORITY 		0
#define FORAGE_PRIORITY		1
#define RELOAD_PRIORITY_WASH 	2
#define NAVIGATE_PRIORITY_GYM 	3 // Changed from reloading to just a navigation
#define NAVIGATE_PRIORITY 4

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.8
/* Threshold to define the battery discharged */
#define BATTERY_WASH_THRESHOLD 0.25
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_GYM_THRESHOLD 0.2
#define NAVIGATE_THRESHOLD 0.5


#define SPEED 1000


/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	m_nWriteToFile = n_write_to_file;
	
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CLightSensor*) m_pcEpuck->GetSensor(SENSOR_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);

   /*Initialize Blue Light max value*/
   fMaxBlueLight = 0.0;

   /*Initialize inhibitors*/
   fForageToWashInhibitor = 1.0;
   fWashToNavigateInhibitor = 1.0;

	justStopped = 0;
	/* Initilize Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;


	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
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

void CIri1Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fForageToWashInhibitor = 1.0;
	fWashToNavigateInhibitor = 1.0;

	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(LED_COLOR_YELLOW);
	
	ObstacleAvoidance ( AVOID_PRIORITY );
	Forage ( FORAGE_PRIORITY );
	GoLoadWash ( RELOAD_PRIORITY_WASH);
	NavigateGym ( NAVIGATE_PRIORITY_GYM);
	Navigate ( NAVIGATE_PRIORITY );
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator ( void )
{
	int nBehavior;
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
			break;
		}
	}

	m_fLeftSpeed = m_fActivationTable[nBehavior][0];
	m_fRightSpeed = m_fActivationTable[nBehavior][1];
	
  printf("%d %2.4f %2.4f \n", nBehavior, m_fLeftSpeed, m_fRightSpeed);
	
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

void CIri1Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	
	/* Leer Sensores de Proximidad */
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

	printf("fMaxProx: %2f\n",fMaxProx);

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Increase time it has collapsed*/
		if(m_fActivationTable[FORAGE_PRIORITY][2] != 1.0){justStopped += 1;}
		printf("JustStoppedAVOIDPost: %2i\n",justStopped);

		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
		

		double fCLinear = 1.0;
		double fCAngular = 1.0;
		double fC1 = SPEED / M_PI;
		
		/* Calc Linear Speed */
		double fVLinear = SPEED * fCLinear * ( cos ( fRepelent / 2) );

		/*Calc Angular Speed */
		double fVAngular = fRepelent;

		m_fActivationTable[un_priority][0] = fVLinear - fC1 * fVAngular;
		m_fActivationTable[un_priority][1] = fVLinear + fC1 * fVAngular;
		m_fActivationTable[un_priority][2] = 1.0;
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

void CIri1Controller::Navigate ( unsigned int un_priority )
{
	/* Leer Sensores de Luz */
	double* redLight = m_seRedLight->GetSensorReading(m_pcEpuck);
	double* blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	double fTotalBlueLight = 0.0;
	double fTotalRedLight = 0.0;

	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	{
		fTotalBlueLight += blueLight[i];

	}

	/* Actualizamos el valor maximo de la luz azul*/
	if( fMaxBlueLight < fTotalBlueLight ){
		fMaxBlueLight = fTotalBlueLight;
	}
	
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Suelo*/
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);

	/* DEBUG */
	/* Leer Battery Sensores */
	printf("fTotalBlueLight: %2f\n",fTotalBlueLight);
	printf("fMaxBlueLight: %2f\n",fMaxBlueLight);
	printf("fTotalRedLight: %2f\n",fTotalRedLight);
	printf("GroundMemorySensor: %2f\n",groundMemory[0]);
	printf("GroundSensor: %2f\n",ground[0]);
	printf("JustStopped: %2i\n",justStopped);
	/* DEBUG */

	
	/* If not in black area and not pointing to the light */
	if ( (ground[0]==1 && groundMemory[0]==0) && justStopped==0 && (blueLight[0]*blueLight[7]==0) && fWashToNavigateInhibitor )//&& (fTotalBlueLight < fMaxBlueLight) )
	{
		/* Go for the light */  
			/*DEBUG*/
			printf("VOY A POR LA LUZ[NAVIGATE] \n");
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);
			/*DEBUG*/

			if(justStopped > 0) {justStopped= justStopped-1;}
			double lightLeft = blueLight[0] + blueLight[1] + blueLight[2] + blueLight[3];
			double lightRight = blueLight[4] + blueLight[5] + blueLight[6] + blueLight[7];
			
			if ( lightLeft > lightRight )
			{
				m_fActivationTable[un_priority][0] = -SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
			else
			{
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = -SPEED;
			}
		
	}else{
		if((justStopped > 0) && (m_fActivationTable[AVOID_PRIORITY][2]!=1.0)) {justStopped= justStopped-1;}
		m_fActivationTable[un_priority][0] = SPEED;
		m_fActivationTable[un_priority][1] = SPEED;
	}

	m_fActivationTable[un_priority][2] = 1.0;
	
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f %2.4f\n", m_fTime, fTotalBlueLight, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}

void CIri1Controller::NavigateGym ( unsigned int un_priority )
{
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	double fTotalLight = 0.0; 
	double tmp[2];

	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
	{
		tmp[0] = light[0] + light[1] + light[2] + light[3];
 		tmp[1] = light[7] + light[6] + light[5] + light[4];

		fTotalLight += light[i];
	}
	
	/* DEBUG */
	/* Leer Luz Total */
	printf("fTotalLight: %2f\n",fTotalLight);
	/* DEBUG */
	
	if ( tmp[0]*fWashToNavigateInhibitor > NAVIGATE_GYM_THRESHOLD )
	{
		tmp[0] = 2 * NAVIGATE_GYM_THRESHOLD - tmp[0];
		m_fActivationTable[un_priority][0] =  SPEED*(0.65 + tmp[0]);
		m_fActivationTable[un_priority][2] = 1.0;
	} 
	if(tmp[1]*fWashToNavigateInhibitor > NAVIGATE_GYM_THRESHOLD){
		tmp[1] = 2 * NAVIGATE_GYM_THRESHOLD - tmp[1];
		m_fActivationTable[un_priority][1] =  SPEED*(0.6 + tmp[1]);
		m_fActivationTable[un_priority][2] = 1.0;
	}
	
	
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateGymOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f %2.4f\n", m_fTime, fTotalLight, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoLoadWash ( unsigned int un_priority )
{
	/* Leer Battery Sensores */
	double* battery = m_seRedBattery->GetSensorReading(m_pcEpuck);
		
	/* Leer Sensores de Luz */
	double* light = m_seRedLight->GetSensorReading(m_pcEpuck);

	/* DEBUG */
	/* Leer Red Battery Sensores de Suelo Memory */
	double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);
	printf("Red Battery: %2f\n",battery[0]);
	/* DEBUG */

	if ( (battery[0]  < BATTERY_WASH_THRESHOLD) && fForageToWashInhibitor==1.0 )
	{
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
		

		fWashToNavigateInhibitor = 0.0; 
		/* If not pointing to the light */
		if ( ( light[0] * light[7] == 0.0 ) )
		{
			m_fActivationTable[un_priority][2] = 1.0;

			double lightLeft 	= light[0] + light[1] + light[2] + light[3];
			double lightRight = light[4] + light[5] + light[6] + light[7];

			if ( lightLeft > lightRight )
			{
				m_fActivationTable[un_priority][0] = -SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
			else
			{
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = -SPEED;
			}
		}
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/redbatteryOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Forage ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	
	/* If with a virtual puck */
	if ( groundMemory[0]  == 1.0 ) //removed inhibitor
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);
		
		fForageToWashInhibitor= 0.0;

		/* DEBUG */
		/* Leer Red Battery Sensores de Suelo Memory */
		printf("ForageToWashInhibitor: %2f\n",fForageToWashInhibitor);
		/* DEBUG */

		/* Go oposite to the light */
		if ( (blueLight[2]+blueLight[3]) * (blueLight[4]+blueLight[5]) == 0.0 ) 
		{
			m_fActivationTable[un_priority][2] = 1.0;

			double lightLeft 	= blueLight[0] + blueLight[1] + blueLight[2] + blueLight[3];
			double lightRight = blueLight[4] + blueLight[5] + blueLight[6] + blueLight[7];

			if ( lightLeft > lightRight )
			{
				m_fActivationTable[un_priority][0] = SPEED;
				m_fActivationTable[un_priority][1] = -SPEED;
			}
			else
			{
				m_fActivationTable[un_priority][0] = -SPEED;
				m_fActivationTable[un_priority][1] = SPEED;
			}
		}
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/forageOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, fForageToWashInhibitor, groundMemory[0], blueLight[0], blueLight[1], blueLight[2], blueLight[3], blueLight[4], blueLight[5], blueLight[6], blueLight[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

