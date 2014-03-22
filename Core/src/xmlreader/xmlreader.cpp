#include "xmlreader/xmlReader.h"

#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <iostream>
#include <sstream>
#include <stdexcept>

//======================= Helper functions for XML reading =======================
xmlDocPtr XMLGetDoc (const char *docname);
xmlXPathObjectPtr XMLSearchForKeyword(xmlDocPtr m_doc, char* m_keyword);
xmlXPathObjectPtr XMLGetNodeSet (xmlDocPtr a_doc, xmlChar *a_xpath);
double XMLGetValue(xmlNodePtr a_node, char* a_attributeName);
void XMLParseDrive(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo);
void XMLParseEncoder(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo);
void XMLParseAnalogIn(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo);
void XMLParseActuator(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo);
void XMLPopulateActuatorIOInfo(xmlDocPtr a_doc, IOInfo &a_ioInfo);
std::string XMLGetRobotType(xmlDocPtr a_doc);
void XMLGetRowValue(xmlNodePtr a_node, char* a_attributeName, double* a_rowValue);
void XMLParseActuatorToJointPosition(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo);
void XMLParseJointToActuatorPosition(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo);
void XMLParseActuatorToJointTorque(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo);
void XMLParseJointToActuatorTorque(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo);
void XMLPopulateCouplingMatrices(xmlDocPtr a_doc, IOInfo &a_ioInfo);

//========================== Public interface functions ==========================
std::string xmlReaderGetRobotType(const char *a_filename)
{
    xmlDocPtr p_document = XMLGetDoc(a_filename);

    if (p_document != NULL)
    {
        std::string returnVal = XMLGetRobotType(p_document);
        xmlFreeDoc(p_document);
        xmlCleanupParser();

        return returnVal;
    }
    else
    {
        std::stringstream s;
        s << "Unable to load XML file: " << a_filename;
        throw std::runtime_error(s.str());

        return "";
    }
}

//--------------------------------------------------------------------------------
void xmlReaderGetRobotInfo(const char *a_filename, IOInfo &a_ioInfo)
{
    xmlDocPtr p_document = XMLGetDoc(a_filename);

    if (p_document != NULL)
    {
        XMLPopulateActuatorIOInfo(p_document, a_ioInfo);
        XMLPopulateCouplingMatrices(p_document, a_ioInfo);
    }
    else
    {
        std::stringstream s;
        s << "Unable to load XML file: " << a_filename;
        throw std::runtime_error(s.str());
    }

    return;
}



//=========================== Private Helper Functions ===========================
xmlDocPtr XMLGetDoc (const char *docname)
{
    xmlDocPtr doc;
    doc = xmlParseFile(docname);

    if (doc == NULL )
    {
        //printf(stderr,"Document not parsed successfully. \n");
        return NULL;
    }

    return doc;
}

//--------------------------------------------------------------------------------
xmlXPathObjectPtr XMLSearchForKeyword(xmlDocPtr m_doc, char* m_keyword)
{
    xmlChar *m_keywordXML = (xmlChar*) m_keyword;
    xmlXPathObjectPtr m_result = XMLGetNodeSet(m_doc, m_keywordXML);

    return m_result;
}

//--------------------------------------------------------------------------------
xmlXPathObjectPtr XMLGetNodeSet (xmlDocPtr a_doc, xmlChar *a_xpath)
{
    xmlXPathContextPtr context;
    xmlXPathObjectPtr result;

    context = xmlXPathNewContext(a_doc);
    if (context == NULL)
    {
        //printf("Error in xmlXPathNewContext\n");
        return NULL;
    }

    result = xmlXPathEvalExpression(a_xpath, context);
    xmlXPathFreeContext(context);
    if (result == NULL)
    {
        //printf("Error in xmlXPathEvalExpression\n");
        return NULL;
    }
    if(xmlXPathNodeSetIsEmpty(result->nodesetval))
    {
        xmlXPathFreeObject(result);
        //printf("No result\n");
        return NULL;
    }
    return result;
}

//--------------------------------------------------------------------------------
double XMLGetValue(xmlNodePtr a_node, char* a_attributeName)
{
    xmlChar *t_value;
    t_value = xmlGetProp(a_node, (xmlChar*)a_attributeName);

    double t_returnValue = atof((char*)t_value);
    xmlFree(t_value);

    return t_returnValue;
}

//--------------------------------------------------------------------------------
void XMLParseDrive(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    while (t_node != NULL)
    {
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"AmpsToBits")))
        {
            // Process AmpsToBits Information
            char *t_offset = new char[10];
            strcpy(t_offset, "Offset" );
            a_ioInfo.m_driveAmpsToBitsOffset[a_actuatorNum] = XMLGetValue(t_node, t_offset);
            delete t_offset;

            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_driveAmpsToBitsScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("offset: %f\t", m_ioInfo.m_driveAmpsToBitsOffset[a_actuatorNum]);
            //printf("scale: %f\n", m_ioInfo.m_driveAmpsToBitsScale[a_actuatorNum]);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"BitsToFeedbackAmps")))
        {
            // Process BitsToFeedbackAmps Information
            char *t_offset = new char[10];
            strcpy(t_offset, "Offset" );
            a_ioInfo.m_driveBitsToFeedbackAmpsOffset[a_actuatorNum] = XMLGetValue(t_node, t_offset);
            delete t_offset;

            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_driveBitsToFeedbackAmpsScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("offset: %4.2f\t", m_ioInfo.m_driveBitsToFeedbackAmpsOffset[a_actuatorNum]);
            //printf("scale: %4.2f\n", m_ioInfo.m_driveBitsToFeedbackAmpsScale[a_actuatorNum]);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"NmToAmps")))
        {
            // Process NmToAmps Information
            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_driveTorqueNmToAmpsScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("scale: %4.2f\n", m_ioInfo.m_driveTorqueNmToAmpsScale[a_actuatorNum]);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"MaxCurrent")))
        {
            // Process MaxCurrent Information
            char* t_value = new char[10];
            strcpy(t_value, "Value");
            a_ioInfo.m_driveMaxAmpsValue[a_actuatorNum] = XMLGetValue(t_node, t_value);
            delete t_value;

            //printf("value: %4.2f\n", m_ioInfo.m_driveMaxAmpsValue[a_actuatorNum]);
        }

        t_node = t_node->next;
    }
}

//--------------------------------------------------------------------------------
void XMLParseEncoder(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    while (t_node != NULL)
    {
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"BitsToPosSI")))
        {
            // Process BitsToPosSI Information
            char* t_offset = new char[10];
            strcpy(t_offset, "Offset");
            a_ioInfo.m_encoderBitsToAngleDegOffset[a_actuatorNum] = XMLGetValue(t_node, t_offset);
            delete t_offset;

            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_encoderBitsToAngleDegScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("offset: %4.2f\t", m_ioInfo.m_encoderBitsToAngleDegOffset[a_actuatorNum]);
            //printf("scale: %4.2f\n", m_ioInfo.m_encoderBitsToAngleDegScale[a_actuatorNum]);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"BitsToDeltaPosSI")))
        {
            // Process BitsToDeltaPosSI Information
            char* t_offset = new char[10];
            strcpy(t_offset, "Offset");
            a_ioInfo.m_encoderBitsToAngleVelocityDegPSecOffset[a_actuatorNum] = XMLGetValue(t_node, t_offset);

            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_encoderBitsToAngleVelocityDegPSecScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("offset: %4.2f\t", m_ioInfo.m_encoderBitsToAngleVelocityDegPSecOffset[a_actuatorNum]);
            //printf("scale: %4.2f\n", m_ioInfo.m_encoderBitsToAngleVelocityDegPSecScale[a_actuatorNum]);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"BitsToDeltaT")))
        {
            // Process BitsToDeltaT Information
            //printf("\t\tBitsToDeltaT:\n");
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"CountsPerTurn")))
        {
            // Process CountsPerTurn Information
            //printf("\t\tCountsPerTurn:\n");
        }

        t_node = t_node->next;
    }
}

//--------------------------------------------------------------------------------
void XMLParseAnalogIn(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    while (t_node != NULL)
    {
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"BitsToVolts")))
        {
            // Process BitsToVolts Information
            char* t_offset = new char[10];
            strcpy(t_offset, "Offset");
            a_ioInfo.m_potentiometerBitsToVoltageOffset[a_actuatorNum] = XMLGetValue(t_node, t_offset);
            delete t_offset;

            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_potentiometerBitsToVoltageScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("offset: %4.2f\t", m_ioInfo.m_potentiometerBitsToVoltageOffset[a_actuatorNum]);
            //printf("scale: %4.2f\n", m_ioInfo.m_potentiometerBitsToVoltageScale[a_actuatorNum]);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"VoltsToPosSI")))
        {
            // Process VoltsToPosSI Information
            char* t_offset = new char[10];
            strcpy(t_offset, "Offset");
            a_ioInfo.m_potentiometerVoltageToAngleDegOffset[a_actuatorNum] = XMLGetValue(t_node, t_offset);
            delete t_offset;

            char* t_scale = new char[10];
            strcpy(t_scale, "Scale");
            a_ioInfo.m_potentiometerVoltageToAngleDegScale[a_actuatorNum] = XMLGetValue(t_node, t_scale);
            delete t_scale;

            //printf("offset: %4.2f\t", m_ioInfo.m_potentiometerVoltageToAngleDegOffset[a_actuatorNum]);
            //printf("scale: %4.2f\n", m_ioInfo.m_potentiometerVoltageToAngleDegScale[a_actuatorNum]);
        }

        t_node = t_node->next;
    }
}

//--------------------------------------------------------------------------------
void XMLParseActuator(xmlDocPtr a_doc, xmlNodePtr a_node, int a_actuatorNum, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    while (t_node != NULL)
    {
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"Drive")))
        {
            // Process Drive Information
            //printf("\tDrive:\n");
            XMLParseDrive(a_doc, t_node, a_actuatorNum, a_ioInfo);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"Encoder")))
        {
            // Process Encoder Information
            //printf("\tEncoder:\n");
            XMLParseEncoder(a_doc, t_node, a_actuatorNum, a_ioInfo);
        }
        else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"AnalogIn")))
        {
            // Process AnaloginInformation
            //printf("\tAnalogIn:\n");
            XMLParseAnalogIn(a_doc, t_node, a_actuatorNum, a_ioInfo);
        }

        t_node = t_node->next;
    }
    return;
}

//--------------------------------------------------------------------------------
std::string XMLGetRobotType(xmlDocPtr a_doc)
{
    xmlXPathObjectPtr t_resultRobot;
    xmlNodeSetPtr nodeset;
    std::string t_returnVal;

    // ---------------------------- Actuator Information -----------------------------------
    char* t_keyword = new char[10];
    strcpy(t_keyword, "//Robot");
    t_resultRobot = XMLSearchForKeyword(a_doc, t_keyword);
    delete t_keyword;

    if (t_resultRobot)
    {
        nodeset = t_resultRobot->nodesetval;
        for (int i = 0; i < nodeset->nodeNr; i++)
        {
            xmlNodePtr cur;
            cur = nodeset->nodeTab[i];

            if ((!xmlStrcmp(cur->name, (const xmlChar *)"Robot")))
            {
                // ------- Get Actuator attributes -------
                xmlChar *t_name;
                t_name = xmlGetProp(cur, (xmlChar*)"Name");

                if (xmlStrcmp(t_name, (xmlChar*)"MTML") == 0)
                {
                    t_returnVal = "MTM";
                }
                else if (xmlStrcmp(t_name, (xmlChar*)"MTMR") == 0)
                {
                    t_returnVal = "MTM";
                }
                else if (xmlStrcmp(t_name, (xmlChar*)"PSM1") == 0)
                {
                    t_returnVal = "PSM";
                }
                else if (xmlStrcmp(t_name, (xmlChar*)"PSM2") == 0)
                {
                    t_returnVal = "PSM";
                }
                else
                {
                    t_returnVal = "Unknown";
                }

                xmlFree(t_name);
            }
        }
    }

    xmlXPathFreeObject (t_resultRobot);

    return t_returnVal;
}

//--------------------------------------------------------------------------------
void XMLPopulateActuatorIOInfo(xmlDocPtr a_doc, IOInfo &a_ioInfo)
{
    xmlXPathObjectPtr t_resultActuator;
    xmlNodeSetPtr nodeset;

    // ---------------------------- Actuator Information -----------------------------------
    char* t_keyword = new char[10];
    strcpy(t_keyword, "//Actuator");
    t_resultActuator = XMLSearchForKeyword(a_doc, t_keyword);
    delete t_keyword;

    if (t_resultActuator)
    {
        nodeset = t_resultActuator->nodesetval;
        for (int i = 0; i < nodeset->nodeNr; i++)
        {
            //keyword = xmlNodeListGetString(doc, nodeset->nodeTab[i]->xmlChildrenNode, 1);
            ////printf("keyword: %s\n", keyword);
            //xmlFree(keyword);

            xmlNodePtr cur;
            cur = nodeset->nodeTab[i];
            if ((!xmlStrcmp(cur->name, (const xmlChar *)"Actuator")))
            {
                // ------- Get Actuator attributes -------
                xmlChar *t_actuatorID;
                t_actuatorID = xmlGetProp(cur, (xmlChar*)"ActuatorID");

                //printf("Actuator ID: %s\t", t_actuatorID);
                xmlFree(t_actuatorID);

                xmlChar *t_axisID;
                t_axisID = xmlGetProp(cur, (xmlChar*)"AxisID");

                //printf("Axis ID: %s\t", t_axisID);
                xmlFree(t_axisID);

                xmlChar *t_boardID;
                t_boardID = xmlGetProp(cur, (xmlChar*)"BoardID");

                //printf("Board ID: %s\t", t_boardID);
                xmlFree(t_boardID);

                xmlChar *t_type;
                t_type = xmlGetProp(cur, (xmlChar*)"Type");
                if (xmlStrcmp(t_type,(xmlChar*)"Revolute") == 0)
                {
                    a_ioInfo.m_jointType[i] = REVOLUTE;
                }
                else
                {
                    a_ioInfo.m_jointType[i] = PRISMATIC;
                }

                xmlFree(t_type);
                XMLParseActuator(a_doc, cur, i, a_ioInfo);
            }
        }
    }

    xmlXPathFreeObject (t_resultActuator);
}
//--------------------------------------------------------------------------------
void XMLGetRowValue(xmlNodePtr a_node, char* a_attributeName, double* a_rowValue)
{
    xmlChar *t_rowValue;
    t_rowValue = xmlGetProp(a_node, (xmlChar*)a_attributeName);

    char * t_char;
    t_char = strtok ((char*)t_rowValue," ");

    int t_colCounter = 0;
    while (t_char != NULL)
    {
        double t_value = atof(t_char);
        t_char = strtok (NULL, " ");

        a_rowValue[t_colCounter] = t_value;
        t_colCounter++;
    }

    xmlFree(t_rowValue);
}

//--------------------------------------------------------------------------------
void XMLParseActuatorToJointPosition(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    int t_counter = 0;

    while (t_node != NULL)
    {
        double t_rowValues[8];
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"Row")))
        {
            char* t_attributeName = new char[10];
            strcpy(t_attributeName, "Val");
            XMLGetRowValue(t_node, t_attributeName, t_rowValues);
            delete t_attributeName;

            for (int i = 0; i < 8; i++)
            {
                a_ioInfo.m_couplingAcutatorToJointPosition[t_counter][i] = t_rowValues[i];
            }

            t_counter = t_counter + 1;
        }
        t_node = t_node->next;
    }

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            //printf("%4.2f ", a_ioInfo.m_couplingAcutatorToJointPosition[i][j]);
        }
        //printf("\n");
    }

    return;
}

//--------------------------------------------------------------------------------
void XMLParseJointToActuatorPosition(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    int t_counter = 0;

    while (t_node != NULL)
    {
        double t_rowValues[8];
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"Row")))
        {
            char* t_attributeName = new char[10];
            strcpy(t_attributeName, "Val");
            XMLGetRowValue(t_node, t_attributeName, t_rowValues);
            delete t_attributeName;

            for (int i = 0; i < 8; i++)
            {
                a_ioInfo.m_couplingJointToActuatorPosition[t_counter][i] = t_rowValues[i];
            }

            t_counter = t_counter + 1;
        }
        t_node = t_node->next;
    }

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            //printf("%4.2f ", m_ioInfo.m_couplingJointToActuatorPosition[i][j]);
        }
        //printf("\n");
    }

    return;
}

//--------------------------------------------------------------------------------
void XMLParseActuatorToJointTorque(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    int t_counter = 0;

    while (t_node != NULL)
    {
        double t_rowValues[8];
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"Row")))
        {
            char* t_attributeName = new char[10];
            strcpy(t_attributeName, "Val");
            XMLGetRowValue(t_node, t_attributeName, t_rowValues);
            delete t_attributeName;

            for (int i = 0; i < 8; i++)
            {
                a_ioInfo.m_couplingAcutatorToJointTorque[t_counter][i] = t_rowValues[i];
            }

            t_counter = t_counter + 1;
        }
        t_node = t_node->next;
    }

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            //printf("%4.2f ", m_ioInfo.m_couplingAcutatorToJointTorque[i][j]);
        }
        //printf("\n");
    }

    return;
}

//--------------------------------------------------------------------------------
void XMLParseJointToActuatorTorque(xmlDocPtr a_doc, xmlNodePtr a_node, IOInfo &a_ioInfo)
{
    xmlNodePtr t_node = a_node->xmlChildrenNode;
    int t_counter = 0;

    while (t_node != NULL)
    {
        double t_rowValues[8];
        if ((!xmlStrcmp(t_node->name, (const xmlChar *)"Row")))
        {
            char* t_attributeName = new char[10];
            strcpy(t_attributeName, "Val");
            XMLGetRowValue(t_node, t_attributeName, t_rowValues);
            delete t_attributeName;

            for (int i = 0; i < 8; i++)
            {
                a_ioInfo.m_couplingJointToActuatorTorque[t_counter][i] = t_rowValues[i];
            }
            t_counter = t_counter + 1;
        }
        t_node = t_node->next;
    }

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            //printf("%4.2f ", m_ioInfo.m_couplingJointToActuatorTorque[i][j]);
        }
        //printf("\n");
    }

    return;
}

//--------------------------------------------------------------------------------
void XMLPopulateCouplingMatrices(xmlDocPtr a_doc, IOInfo &a_ioInfo)
{
    xmlXPathObjectPtr t_resultCoupling;
    xmlNodeSetPtr nodeset;

    // ---------------------------- Actuator Information -----------------------------------
    char* t_keyword = new char[10];
    strcpy(t_keyword, "//Coupling");
    t_resultCoupling = XMLSearchForKeyword(a_doc, t_keyword);
    delete t_keyword;

    if (t_resultCoupling)
    {
        nodeset = t_resultCoupling->nodesetval;
        xmlNodePtr cur = nodeset->nodeTab[0];
        xmlNodePtr t_node = cur->xmlChildrenNode;
        while (t_node != NULL)
        {
            if ((!xmlStrcmp(t_node->name, (const xmlChar *)"ActuatorToJointPosition")))
            {
                // Process ActuatorToJointPosition Information
                //printf("ActuatorToJointPosition:\n\n");
                XMLParseActuatorToJointPosition(a_doc, t_node, a_ioInfo);
                //printf("\n");
            }
            else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"JointToActuatorPosition")))
            {
                // Process JointToActuatorPosition Information
                //printf("JointToActuatorPosition:\n\n");
                XMLParseJointToActuatorPosition(a_doc, t_node, a_ioInfo);
                //printf("\n");
            }
            else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"ActuatorToJointTorque")))
            {
                // Process ActuatorToJointTorque Information
                //printf("ActuatorToJointTorque:\n\n");
                XMLParseActuatorToJointTorque(a_doc, t_node, a_ioInfo);
                //printf("\n");
            }
            else if ((!xmlStrcmp(t_node->name, (const xmlChar *)"JointToActuatorTorque")))
            {
                // Process JointToActuatorTorque Information
                //printf("JointToActuatorTorque:\n");
                XMLParseJointToActuatorTorque(a_doc, t_node, a_ioInfo);
                //printf("\n");
            }

            t_node = t_node->next;
        }

    }

    xmlXPathFreeObject (t_resultCoupling);
}
