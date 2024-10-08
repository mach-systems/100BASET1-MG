/******************************************************************************
* (c) NXP B.V. 2016 - 2017. All rights reserved.
*
* Disclaimer
* 1. The NXP Software/Source Code is provided to Licensee "AS IS" without any
* warranties of any kind. NXP makes no warranties to Licensee and shall not
* indemnify Licensee or hold it harmless for any reason related to the NXP
* Software/Source Code or otherwise be liable to the NXP customer. The NXP
* customer acknowledges and agrees that the NXP Software/Source Code is
* provided AS-IS and accepts all risks of utilizing the NXP Software under the
* conditions set forth according to this disclaimer.
*
* 2. NXP EXPRESSLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING, BUT
* NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, AND NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS. NXP
* SHALL HAVE NO LIABILITY TO THE NXP CUSTOMER, OR ITS SUBSIDIARIES, AFFILIATES,
* OR ANY OTHER THIRD PARTY FOR ANY DAMAGES, INCLUDING WITHOUT LIMITATION,
* DAMAGES RESULTING OR ALLEGDED TO HAVE RESULTED FROM ANY DEFECT, ERROR OR
* OMMISSION IN THE NXP SOFTWARE/SOURCE CODE, THIRD PARTY APPLICATION SOFTWARE
* AND/OR DOCUMENTATION, OR AS A RESULT OF ANY INFRINGEMENT OF ANY INTELLECTUAL
* PROPERTY RIGHT OF ANY THIRD PARTY. IN NO EVENT SHALL NXP BE LIABLE FOR ANY
* INCIDENTAL, INDIRECT, SPECIAL, EXEMPLARY, PUNITIVE, OR CONSEQUENTIAL DAMAGES
* (INCLUDING LOST PROFITS) SUFFERED BY NXP CUSTOMER OR ITS SUBSIDIARIES,
* AFFILIATES, OR ANY OTHER THIRD PARTY ARISING OUT OF OR RELATED TO THE NXP
* SOFTWARE/SOURCE CODE EVEN IF NXP HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGES.
*
* 3. NXP reserves the right to make changes to the NXP Software/Sourcecode any
* time, also without informing customer.
*
* 4. Licensee agrees to indemnify and hold harmless NXP and its affiliated
* companies from and against any claims, suits, losses, damages, liabilities,
* costs and expenses (including reasonable attorney's fees) resulting from
* Licensee's and/or Licensee customer's/licensee's use of the NXP
* Software/Source Code.
*****************************************************************************/

/**
*
* \file NXP_SJA1105P_diagnostics.c
*
* \author NXP Semiconductors
*
* \date 2017-03-02
*
* \brief Obtain diagnostic information on the switch
*
*****************************************************************************/

/******************************************************************************
* INCLUDES
*****************************************************************************/

#include "typedefs.h"

#include "NXP_SJA1105P_diagnostics.h"
#include "NXP_SJA1105P_switchCore.h"
#include "NXP_SJA1105P_config.h"
#include "NXP_SJA1105P_auxiliaryConfigurationUnit.h"

/******************************************************************************
* DEFINES
*****************************************************************************/

#define N_TRESHOLDS ((uint8_t) SJA1105P_e_tempThreshold_POSITIVE135P5 - (uint8_t) SJA1105P_e_tempThreshold_NOT_VALID)

/******************************************************************************
* INTERNAL TYPE DEFINITIONS
*****************************************************************************/

typedef uint8_t (*get64bitCounter_cb_t) (uint64_t *p_counterValue, uint8_t port, uint8_t switchId);
typedef uint8_t (*get32bitCounter_cb_t) (uint32_t *p_counterValue, uint8_t port, uint8_t switchId);

/******************************************************************************
* INTERNAL FUNCTION DECLARATIONS
*****************************************************************************/

static uint8_t calculate64bitCounter(uint64_t *p_counterValue, get64bitCounter_cb_t pf_getEgressCounter, get64bitCounter_cb_t pf_getIngressCounter, SJA1105P_etherStatDirection_t direction, const SJA1105P_port_t *kp_physicalPort);
static uint8_t calculate32bitCounter(uint32_t *p_counterValue, get32bitCounter_cb_t pf_getEgressCounter, get32bitCounter_cb_t pf_getIngressCounter, SJA1105P_etherStatDirection_t direction, const SJA1105P_port_t *kp_physicalPort);

static SJA1105P_macLevelErrors_t macLevelErrors[SJA1105P_N_LOGICAL_PORTS];
/******************************************************************************
* FUNCTIONS
*****************************************************************************/

/**
* \brief Retrieve a 64-bit Ethernet statistic counter value
*
* \param[in]  counterName Name of the statistic counter requested
* \param[in]  port Number of the port where the statistics are requested
* \param[in]  direction Direction (Ingress/Egress/both) for which the counter value is requested
* \param[out] p_counterValue Pointer to the value of the counter
*
* \return uint8_t Returns 0 upon success, else failed
*/
extern uint8_t SJA1105P_get64bitEtherStatCounter(SJA1105P_etherStat64_t counterName, uint64_t *p_counterValue, uint8_t port, SJA1105P_etherStatDirection_t direction)
{
	uint8_t ret = 1;
	SJA1105P_port_t physicalPort;

	if (SJA1105P_getPhysicalPort(port, &physicalPort) == 0U)
	{
		switch (counterName)
		{
			case SJA1105P_e_etherStat64_N_OCTETS:
				ret = calculate64bitCounter(p_counterValue, SJA1105P_getHlDiagnCounters10, SJA1105P_getHlDiagnCounters12, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat64_N_PKTS:
				ret = calculate64bitCounter(p_counterValue, SJA1105P_getHlDiagnCounters11, SJA1105P_getHlDiagnCounters13, direction, &physicalPort);
				break;
			default:
				break;
		}
	}

	return ret;
}

/**
* \brief Retrieve a 32-bit Ethernet statistic counter value
*
* \param[in]  counterName Name of the statistic counter requested
* \param[in]  port Number of the port where the statistics are requested
* \param[in]  direction Direction (Ingress/Egress/both) for which the counter value is requested
* \param[out] p_counterValue Pointer to the value of the counter
*
* \return uint8_t Returns 0 upon success, else failed
*/
extern uint8_t SJA1105P_get32bitEtherStatCounter(SJA1105P_etherStat32_t counterName, uint32_t *p_counterValue, uint8_t port, SJA1105P_etherStatDirection_t direction)
{
	uint8_t ret = 1;
	SJA1105P_port_t physicalPort;

	if (SJA1105P_getPhysicalPort(port, &physicalPort) == 0U)
	{
		switch (counterName)
		{
			case SJA1105P_e_etherStat32_N_NOT_REACH:
				ret = SJA1105P_getHlDiagnCounters20(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_EGR_DISABLED:
				ret = SJA1105P_getHlDiagnCounters21(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_PART_DROP:
				ret = SJA1105P_getHlDiagnCounters22(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_QFULL:
				ret = SJA1105P_getHlDiagnCounters23(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_POLERR:
				ret = SJA1105P_getHlDiagnCounters14(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_CRCERR:
				ret = SJA1105P_getHlDiagnCounters15(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_SIZEERR:
				ret = SJA1105P_getHlDiagnCounters16(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_VLANERR:
				ret = SJA1105P_getHlDiagnCounters17(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_N664ERR:
				ret = SJA1105P_getHlDiagnCounters18(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;

			case SJA1105P_e_etherStat32_N_ADDR_NOT_LEARNED_DROP:
				ret = SJA1105P_getEthStatCounters22(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_EMPTY_ROUTE_DROP:
				ret = SJA1105P_getEthStatCounters21(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_ILLEGAL_DOUBLE_DROP:
				ret = SJA1105P_getEthStatCounters20(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_DOUBLE_TAGGED_DROP:
				ret = SJA1105P_getEthStatCounters19(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_SINGLE_OUTER_DROP:
				ret = SJA1105P_getEthStatCounters18(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_SINGLE_INNER_DROP:
				ret = SJA1105P_getEthStatCounters17(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;
			case SJA1105P_e_etherStat32_N_UNTAGGED_DROP:
				ret = SJA1105P_getEthStatCounters16(p_counterValue, physicalPort.physicalPort, physicalPort.switchId);
				break;

			case SJA1105P_e_etherStat32_N_PKTS_1024_TO_1518_OCTETS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters15, SJA1105P_getEthStatCounters07, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_PKTS_512_TO_1023_OCTETS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters14, SJA1105P_getEthStatCounters06, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_PKTS_256_TO_511_OCTETS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters13, SJA1105P_getEthStatCounters05, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_PKTS_128_TO_255_OCTETS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters12, SJA1105P_getEthStatCounters04, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_PKTS_65_TO_127_OCTETS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters11, SJA1105P_getEthStatCounters03, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_PKTS_64_OCTETS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters10, SJA1105P_getEthStatCounters02, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_MULTICAST_PKTS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters09, SJA1105P_getEthStatCounters01, direction, &physicalPort);
				break;
			case SJA1105P_e_etherStat32_N_BROADCAST_PKTS:
				ret = calculate32bitCounter(p_counterValue, SJA1105P_getEthStatCounters08, SJA1105P_getEthStatCounters00, direction, &physicalPort);
				break;

			default:
				break;
		}
	}

	return ret;
}

static uint8_t calculate64bitCounter(uint64_t *p_counterValue, get64bitCounter_cb_t pf_getEgressCounter, get64bitCounter_cb_t pf_getIngressCounter, SJA1105P_etherStatDirection_t direction, const SJA1105P_port_t *kp_physicalPort)
{
	uint8_t ret;
	uint64_t tmp;

	switch (direction)
	{
		case SJA1105P_e_etherStatDirection_INGRESS:
			ret = pf_getIngressCounter(p_counterValue, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			break;
		case SJA1105P_e_etherStatDirection_EGRESS:
			ret = pf_getEgressCounter(p_counterValue, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			break;
		default:  /* BOTH */
			ret  = pf_getIngressCounter(p_counterValue, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			ret += pf_getEgressCounter(&tmp, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			*p_counterValue = (uint64_t) (*p_counterValue + tmp);
			break;
	}

	return ret;
}

static uint8_t calculate32bitCounter(uint32_t *p_counterValue, get32bitCounter_cb_t pf_getEgressCounter, get32bitCounter_cb_t pf_getIngressCounter, SJA1105P_etherStatDirection_t direction, const SJA1105P_port_t *kp_physicalPort)
{
	uint8_t ret;
	uint32_t tmp;

	switch (direction)
	{
		case SJA1105P_e_etherStatDirection_INGRESS:
			ret = pf_getIngressCounter(p_counterValue, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			break;
		case SJA1105P_e_etherStatDirection_EGRESS:
			ret = pf_getEgressCounter(p_counterValue, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			break;
		default:  /* BOTH */
			ret  = pf_getIngressCounter(p_counterValue, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			ret += pf_getEgressCounter(&tmp, kp_physicalPort->physicalPort, kp_physicalPort->switchId);
			*p_counterValue = (uint32_t) (*p_counterValue + tmp);
			break;
	}
	return ret;
}

/**
* \brief Retrieve the mac level errors of a port.
*
* \param[in]  p_macLevelErrors 
* \param[in]  port Number of the port where the statistics are requested
*
* \return uint8_t Returns 0 upon success, else failed
*/
extern uint8_t SJA1105P_getMacErrors(SJA1105P_macLevelErrors_t *p_macLevelErrors, uint8_t port)
{
	uint8_t ret = 1;
	SJA1105P_macLevelDiagnCountersArgument_t macLevelDiagnCountersArgument;
	SJA1105P_port_t physicalPort;

	if (SJA1105P_getPhysicalPort(port, &physicalPort) == 0U)
	{
		ret = SJA1105P_getMacLevelDiagnCounters(&macLevelDiagnCountersArgument, physicalPort.physicalPort, physicalPort.switchId);
		/* accumulate MAC errors. The hardware register is cleared on read */
		macLevelErrors[port].nRunt     += (uint32_t) macLevelDiagnCountersArgument.nRunt;
		macLevelErrors[port].nSoferr   += (uint32_t) macLevelDiagnCountersArgument.nSoferr;
		macLevelErrors[port].nAlignerr += (uint32_t) macLevelDiagnCountersArgument.nAlignerr;
		macLevelErrors[port].nMiierr   += (uint32_t) macLevelDiagnCountersArgument.nMiierr;
		*p_macLevelErrors = macLevelErrors[port];
	}

	return ret;
}

/**
* \brief Retrieve the temperature sensor reading from the switches
*
* This function will use the built in temperature sensor to determine
* the rough temperature. Not that this is only approximate up to 
* around 5 degrees due to the sensor quantization.
*
* \param[out] a_temperature Array containing the readings
*
* \return uint8_t Returns 0 upon success, else failed
*/
extern uint8_t SJA1105P_getTemperature(SJA1105P_tempThreshold_t a_temperature[SJA1105P_N_SWITCHES])
{
	uint8_t ret = 0;
	uint8_t switchId;
	uint8_t i;
	SJA1105P_tsConfigArgument_t tempSensor;
	uint8_t exceeded;
	tempSensor.pd = 0;  /* sensor active */
	for (switchId = 0; switchId < SJA1105P_N_SWITCHES; switchId++)
	{
		tempSensor.threshold = SJA1105P_e_tempThreshold_NOT_VALID;  /* minimum temperature */
		for (i = 0; i < N_TRESHOLDS; i++)
		{
			tempSensor.threshold = SJA1105P_convertToTempThreshold((uint32_t) (tempSensor.threshold) + 1U);
			ret += SJA1105P_setTsConfig(&tempSensor, switchId);
			ret += SJA1105P_getTsStatus(&exceeded, switchId);
			if (exceeded == 0U)
			{
				a_temperature[switchId] = tempSensor.threshold;
				break;
			}
		}
	}

	return ret;
}
