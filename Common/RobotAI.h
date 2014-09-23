#pragma once	// Only include this header once

#include "RobotConfig.h"
#include "Config.h"
#if AI_ENGINE_ENABLED == 1

#include <rebecca\network_all.h>
#include <rebecca\all.h>
#include <string>
using namespace rebecca;

class CustomCallBacks : public CallBacks
{
	private:
		/**
		 * Keep our userId so we can filter
		 * and only show callbacks to our 
		 * user Id
		 */
		StringPimpl m_userId;

		/**
		 * Keep our userId so we can filter
		 * and only show callbacks to our 
		 * botId
		 */
		StringPimpl m_botId;

		/**
		 * Keep our userId so we can filter
		 * and only show callbacks to our 
		 * endUserId
		 */
		StringPimpl m_endUserId;

public:
	CustomCallBacks(StringPimpl userId, StringPimpl botId, StringPimpl endUserId)
			: m_userId(userId), 
			  m_botId(botId), 
			  m_endUserId(endUserId)
		{ }

	void storeGossip(const char * const gossip); 
	void categoryLoaded();
	void filePreLoad(const char * const fileName);
	void filePostLoad(const char * const fileName);
	void symbolicReduction(const char * const symbol);
	void infiniteSymbolicReduction();
	virtual void XMLParseError(const char * const message); 
	virtual void XMLParseWarning(const char * const message);
	virtual void XMLParseFatalError(const char * const message); 
	virtual void thatStarTagSizeExceeded(); 
	virtual void topicStarTagSizeExceeded(); 
	virtual void starTagSizeExceeded(); 
	virtual void inputTagNumericConversionError(const char * const message); 
	virtual void inputTagSizeExceeded(); 
	virtual void starTagNumericConversionError(const char * const message); 
	virtual void thatTagSizeExceeded(); 
	virtual void thatTagNumericConversionError(const char * const message); 
	virtual void topicStarTagNumericConversionError(const char * const message); 
	virtual void thatStarTagNumericConversionError(const char * const message); 

};


class CRobotAI
{

public:
	CRobotAI();
	~CRobotAI(){};


	// Member valiables
private:
	CString				 m_BotName;
	NetworkAimlFacade	*m_pAiml;
	NetworkGraphBuilder	*m_pBuilder;
	CustomCallBacks		*m_pCustomCallBack;

	CString m_ConfigurationDirectory;
	CString m_SubstitutionsFile;
	CString m_SentenceSplittersFile;
	CString m_PropertiesFile;

	CString m_AimlDirectory;
	CString m_AimlSchemaPath;
	CString m_CommonTypesSchemaPath;
	CString m_BotConfigurationSchemaPath;

// Functions
public:
	int Initialize();
	void GetAIResponse( CString input, CString &strResponse );

	void Test();

};	// End CRobotAI


#endif // AI_ENGINE_ENABLED == 1

