// This calls the Rebecca Engine, a variant of A.L.I.C.E 

// RebeccaAIML, Artificial Intelligence Markup Language 
// C++ api and engine, Copyright (C) 2005 Frank Hassanabad
// See all.h for LPGL license info.

#include "stdafx.h"
#include "globals.h"
#if ENABLE_ROBOT_AI == 1 // To disable, see AI_ENGINE_ENABLED in Config.h

//Rebecca includes
#include <rebecca\network_all.h>
#include <rebecca\all.h>

//Std includes
#include <iostream>
#include <string>
using namespace std;

using namespace rebecca;
#include "RobotAI.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


CString MsgString;

// Custom CallBacks class.
// This class inherits from Rebecca's CallBacks class and implements the 
// callbacks for error reporting and informational reporting purposes.
// Although, I pain stakenly reimplemnt every single method, you don't have to.
// Just pick and choose which ones you want to implement, and ignore the rest.  
// All of the methods of CallBacks has a default do nothing implementation.

	// This is called for each AIML "Gossip" tag.
	// I am just printing out the gossip. You can do other things like store it 
	// in a file and then reload the file at startup as a type of persistance.
	// \param gossip The gossip sent to be stored as you see fit
	void CustomCallBacks::storeGossip(const char * const gossip) 
	{ 
		MsgString = "AI:  GOT GOSSIP! (%s)", gossip;
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

	//	ROBOT_LOG( TRUE, "[AI: Gossip: %s\n", gossip)
	}

	// This is called for each AIML category that is loaded into memory.  
	// Loadtime occurs whenver the call GraphBuilder::createGraph() is made.
	// For every 5000 categories loaded into Rebecca's internal data structure 
	// this prints a output message about it.
	void CustomCallBacks::categoryLoaded()
	{
		static int i = 0;
		const int numberOfCategories = 5000;
		
		// Clever way to say if "i" is a multiple of 5000 then print out the number of 
		// categories loaded so far.
		
		if(++i % numberOfCategories == 0)
		{
			ROBOT_LOG( TRUE, "[AI: %d categories loaded]\n", i)
		}
	}

	// Before each AIML file is parsed this method is called.
	// \param fileName The name of the file about to be parsed.
	void CustomCallBacks::filePreLoad(const char * const fileName)
	{
		ROBOT_LOG( TRUE, "[AI: Loading      %s\n", fileName)
	}
	
	// After each AIML file is parsed, this method is called.
	// \param fileName The name of the file just parsed.
	void CustomCallBacks::filePostLoad(const char * const fileName)
	{
		ROBOT_LOG( TRUE, "[AI: Done loading %s\n", fileName)
	}

	// When the "srai" AIML tag is called, the text is sent to this method.
	// Usually refered to as symbolic reduction, you  can see what text is being re-fed back into the 
	// AIML GraphBuilder::getResponse() by AIML its self.
	// \param symbol The text which is being sent back into GraphBuilder::getResponse().
	void CustomCallBacks::symbolicReduction(const char * const symbol)
	{
		ROBOT_LOG( TRUE, "Symbolic reduction: %s\n", symbol)
	}

	// A infinite symbolic reduction has occured and has been terminated.
	// This method is called when symbolic reduction ends up in a infinite loop 
	// and has been terminated.  This is just to alert you to the fact.
	void CustomCallBacks::infiniteSymbolicReduction()
	{
		ROBOT_LOG( TRUE, "[AI: Infinite Symbolic reduction detected]\n")
	}
	
	// Sends you a message about a XMLParseError.
	// Either with AIML files or RebeccaAIML configuration files.
	// \param message The human readable message.
	void CustomCallBacks::XMLParseError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE, "AI XMLParseError: %s\n", message)
		ROBOT_ASSERT(0);
	} 

	// Sends you a message about a XMLParseWarning. 
	// Either with AIML files or RebeccaAIML configuration files.
	// \param message The human readable message.
	void CustomCallBacks::XMLParseWarning(const char * const message)
	{ 
		ROBOT_LOG( TRUE, "AI XMLParseWarning: %s\n", message)
		ROBOT_ASSERT(0);
	} 

	// Sends you a message about a XMLParseFatalError. 
	// Either with AIML files or RebeccaAIML configuration files.
	// \param message The human readable message.
	void CustomCallBacks::XMLParseFatalError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE, "AI XMLParseFatalError: %s\n", message)
		ROBOT_ASSERT(0);
	} 
	
	// During runtime, the "thatStar" AIML tag's size is  exceeded.
	// Runtime is during a call to GraphBuilder::getResponse()
	void CustomCallBacks::thatStarTagSizeExceeded() 
	{ 
		ROBOT_LOG( TRUE, "[AI: Warning thatStar Tag Size Exceeded]\n")
		ROBOT_ASSERT(0);
	}
	
	// During runtime, the "topicStar" AIML tag's size is  exceeded.
	// Runtime is during a call to GraphBuilder::getResponse()
	void CustomCallBacks::topicStarTagSizeExceeded() 
	{ 
		ROBOT_LOG( TRUE, "[AI: Warning topicStar Tag Size Exceeded]\n")
		ROBOT_ASSERT(0);
	}

	// During runtime, the "star" AIML tag's size is  exceeded.
	// Runtime is during a call to GraphBuilder::getResponse()
	void CustomCallBacks::starTagSizeExceeded() 
	{ 
		ROBOT_LOG( TRUE, "[AI: Warning star Tag Size Exceeded]\n")
		//ROBOT_ASSERT(0);
	}

	// A AIML "Input" tag has a non number in its index attribute.
	// This method will only be called during loadtime, GraphBuilder::createGraph().
	// \param message The human readable message.
	void CustomCallBacks::inputTagNumericConversionError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE,"inputTagNuermicConversionError: %s\n", message)
		ROBOT_ASSERT(0);
	}

	// During runtime, the "input" AIML tag's size is exceeded.
	// Runtime is during a call to GraphBuilder::getResponse()
	void CustomCallBacks::inputTagSizeExceeded() 
	{ 
		ROBOT_LOG( TRUE,"[AI: Warning input Tag Size Exceeded]\n")
		ROBOT_ASSERT(0);
	} 

	// A AIML "Star" tag has a non number in its index attribute.
	// This method will only be called during loadtime, GraphBuilder::createGraph().
	// \param message The human readable message.
	void CustomCallBacks::starTagNumericConversionError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE,"starTagNuermicConversionError: %s\n", message)
		//ROBOT_ASSERT(0);
	} 

	// During runtime, the "that" AIML tag's size is exceeded.
	// Runtime is during a call to GraphBuilder::getResponse()
	void CustomCallBacks::thatTagSizeExceeded() 
	{ 
		ROBOT_LOG( TRUE,"[AI: Warning that Tag Size Exceeded]\n")
		//ROBOT_ASSERT(0);
	}

	// A AIML "That" tag has a non number in its index attribute.
	// This method will only be called during loadtime, GraphBuilder::createGraph().
	// \param message The human readable message.
	void CustomCallBacks::thatTagNumericConversionError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE,"thatTagNumericConversionError:%s\n", message)
		//ROBOT_ASSERT(0);
	} 

	// A AIML "TopicStar" tag has a non number in its index attribute.
	// This method will only be called during loadtime, GraphBuilder::createGraph().
	// \param message The human readable message.
	void CustomCallBacks::topicStarTagNumericConversionError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE,"topicStarTagNumericConversionError: %s\n", message)
		//ROBOT_ASSERT(0);
	} 
	
	// A AIML "thatStar" tag has a non number in its index attribute.
	// This method will only be called during loadtime, GraphBuilder::createGraph().
	// \param message The human readable message.
	void CustomCallBacks::thatStarTagNumericConversionError(const char * const message) 
	{ 
		ROBOT_LOG( TRUE,"thatStarTagNumericConversionError: %s\n", message)
		//ROBOT_ASSERT(0);
	}


////////////////////////////////////////////////////////////////////////////////////
CRobotAI::CRobotAI()
{
	// Initialize AI member variables

	// This is responsible for memory management of GraphBuilder.
	int argc = 0;	
	char* args[16];
	 m_pAiml = new NetworkAimlFacade(argc, args);

	// Get the GraphBuilder concrete class that was created inside of AimlFacade.
	// DO NOT try to delete GraphBuilder.  Let AimlFacade handle that when it falls out 
	// of scope.
	m_pBuilder = &(m_pAiml->getNetworkGraphBuilder() );
/*
	// The location of a AIML directory.
	m_AimlDirectory =				"../AI/aiml/loki";

	// The location of the AI configuration directory.  
	// default is current working directory + "../../conf".  
	// This is where needed configuration files are stored.
	CString ConfigDir =		"../AI/conf";
	CString SchemaDir =	"../AI/resources/schema";	// Relative to the conf dir! (go figure!)
//	CString SchemaDir =	"../resources/schema/";	// Relative to the conf dir! (go figure!)

	m_SubstitutionsFile =			ConfigDir + "/substitutions.xml";
	m_SentenceSplittersFile =		ConfigDir + "/sentence-splitters.xml";
	m_PropertiesFile =				ConfigDir + "/properties.xml";

	// The path to the aiml xsd file.	// 
	m_AimlSchemaPath =				SchemaDir + "/AIML.xsd";
//	CheckFileExists( m_AimlSchemaPath );

	// The path to the the common types schema file.
	m_CommonTypesSchemaPath =		SchemaDir + "/common-types.xsd";
//	CheckFileExists( m_CommonTypesSchemaPath );

	// The path to the bot configuration schema file. 
	m_BotConfigurationSchemaPath =	SchemaDir + "/bot-configuration.xsd";
//	CheckFileExists( m_BotConfigurationSchemaPath );

	// This is responsible for memory management of GraphBuilder.
	 NetworkAimlFacade Aiml(argc, args);

	// Create an instantiation of our custom callbacks we created above.
	//CustomCallBacks callback;
*/
}

int CRobotAI::Initialize()
{
	CString MsgString;

	try
	{
		//StringPimpl directoryName; 
		StringPimpl userId = "default";
		StringPimpl botId = "default";
		StringPimpl endUserId = "default";
	
		//set our locale to english
		locale::global(locale("ENG"));

		/*
		 * This is responsible for memory management of 
		 * GraphBuilder.
		 */
//		NetworkAimlFacade aiml(argc, args);


		/*
		 * Set the UserId, botId, endUserId
		 * By default if the user doesn't change this it 
		 * will be "default" for all three
		 */
		m_pBuilder->setUserIdBotIdEndUserId(userId, botId, endUserId);
		/*
		 * Create an instantiation of our custom 
		 * callbacks we created above.
		 *
		 * Hand it our userId, botId, and endUserId so we
		 * can filter based on them and only show callbacks
		 * based on our userId, botId, and endUserId
		 */
		m_pCustomCallBack =  new CustomCallBacks(userId, botId, endUserId);

		/* Give the address to Rebecca for usesage.
		 * Rebecca DOES NOT delete it.  
		 */
		m_pBuilder->setCallBacks(m_pCustomCallBack);


		/*
		 * Get the number of AIML categories loaded in total.
		 */
		int size = m_pBuilder->getSize();

		//Print out the number of categories loaded.
		ROBOT_LOG( TRUE,"AI: %d categories loaded\n", size )

		/*
		 * Get the botName in the configuration file properties.xml
		 * which we parsed above.
		 */
		string botName = m_pBuilder->getBotPredicate("name").c_str();

		/*
		 * Send a initial conversation of "connect" to 
		 * annotated alice and get the response.
		 */
//		StringPimpl response = m_pBuilder->getResponse("connect");		

		//Send the initial opening line of the bot
//		ROBOT_LOG( TRUE, "%s says: %s\n", botName, response.c_str() )
		
		//Send the initial opening line of the bot
//		ROBOT_LOG( TRUE,"AI: %s Started\n", m_BotName)


		// TODO =====================================================
		// Send a initial conversation of "connect" to 
		// annotated alice and get the response.
		//StringPimpl response = m_pBuilder->getResponse("connect");		
		CString strResponse;
		CString strInput;
		strInput = "connect";
		ROBOT_LOG( TRUE,"AI: GetAIResponse (inital response)\n")
		GetAIResponse( strInput, strResponse );

		//Send the initial opening line of the bot
		ROBOT_LOG( TRUE,"AI: %s Started\n", m_BotName)

		MsgString = "Loki AI Startup: " + strResponse + "\n";
		ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )

	}
	catch(NetworkException &e)
	{
		cout << "[NetworkException Found Terminating]" << endl;
		cout << "[" << e.what() << "]" << endl;
		return 1;
	}
	catch(Exception &e)
	{
		cout << "[An uknown exception occured, Terminating program]" << endl;
		cout << "[" << e.what() << "]";
		return 1;
	}
	return 1;
}

void CRobotAI::Test()
{
	ROBOT_LOG( TRUE,"test!\n")
}

void CRobotAI::GetAIResponse( CString strInput, CString &strResponse )
{ 
	CString MsgString;
	try
	{	

	ROBOT_LOG( TRUE,"User Entered: %s\n", strInput)

	//Here we get some internal AI information.
//	ROBOT_LOG( TRUE,"AI Internal information:\n")
//	ROBOT_LOG( TRUE,"=====================\n")
//	ROBOT_LOG( TRUE,"For: \"%s\" : ", strInput)
//	ROBOT_LOG( TRUE,"%s : %s\n",	m_pBuilder->getThat().c_str(), m_pBuilder->getTopic().c_str() )
//	ROBOT_LOG( TRUE,"=====================\n")


	// Finally, get the response back from the AI.
	StringPimpl response = m_pBuilder->getResponse( (LPCTSTR)strInput );
	strResponse = response.c_str();

	ROBOT_LOG( TRUE,"AI DEBUG: AI Response: \"%s\"\n", strResponse )

	//Don't Print out what the AI says. (handled elsewhere)
//	MsgString = "AI says: " + strResponse;
//	ROBOT_DISPLAY( TRUE,  (LPCTSTR)MsgString )


	}
	catch(NetworkException &e)
	{
//		cout << "[NetworkException Found Terminating]" << endl;
		ROBOT_LOG( TRUE, "AI: NetworkException! %s\n", e.what() )
		ASSERT(0);
	}
	catch(Exception &e)
	{
//		cout << "[An uknown exception occured, Terminating program]" << endl;
		ROBOT_LOG( TRUE, "AI: An uknown exception occured! %s\n", e.what() )
		ASSERT(0);
	}
}

#endif // if ENABLE_ROBOT_AI == 1

