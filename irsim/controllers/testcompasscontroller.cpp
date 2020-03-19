
/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "testcompasscontroller.h"



extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CTestCompassController::CTestCompassController (const char* pch_name, CEpuck* pc_epuck) : CController (pch_name, pc_epuck)

{
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor(SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor(SENSOR_COMPASS);

  m_fOrientation = 0.0;
  m_vPosition.x = 0.0;
  m_vPosition.y = 0.0;

  maxErrorA.x = 0.0;
  maxErrorA.y = 0.0;
}

/******************************************************************************/
/******************************************************************************/

CTestCompassController::~CTestCompassController()
{
}


/******************************************************************************/
/******************************************************************************/

void CTestCompassController::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{

	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);

	printf("Encoder Sensor Value: ");
	for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i++)
	{
		printf("%2f ", encoder[i]);
	}
	printf("\n");
  
  printf("Compass sensor value: ");
	for ( int i = 0 ; i < m_seCompass->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", compass[i]);
	}
	printf("\n");

  m_acWheels->SetSpeed(500,300);


}

/******************************************************************************/
/******************************************************************************/

