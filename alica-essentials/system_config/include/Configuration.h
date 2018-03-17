#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

using namespace std;

#include <algorithm>
#include <cstdarg>
#include <cstdarg>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <string>
#include <vector>
//#include "boost/lexical_cast.hpp"

#include "ConfigNode.h"

namespace supplementary
{
class Configuration
{

  protected:
    static const char LIST_ELEMENT_SEPERATOR = ',';
    string filename;
    void collect(ConfigNode *node, vector<string> *params, size_t offset, vector<ConfigNode *> *result);
    void collectSections(ConfigNode *node, vector<string> *params, size_t offset, vector<ConfigNode *> *result);
    string pathNotFound(vector<string> *params);

    ConfigNodePtr configRoot;

    void serialize_internal(ostringstream *ss, ConfigNode *node);
    void serialize_without_root(ostringstream *ss, ConfigNode *node);

    template <typename Target>
    Target convert(string value)
    {
        string errMsg = "Configuration: Type not handled! Value to be converted was: " + value;
        cerr << errMsg << endl;
        throw new runtime_error(errMsg);
    }

    template <typename Target>
    vector<Target> convertList(string value)
    {
        string errMsg = "Configuration: List Type not handled! Value to be converted was: " + value;
        cerr << errMsg << endl;
        throw new runtime_error(errMsg);
    }

  public:
    Configuration();
    Configuration(string filename);
    Configuration(string filename, const string content);

    inline void load(string filename)
    {
        load(filename, shared_ptr<ifstream>(new ifstream(filename.c_str(), ifstream::in)), false, false);
    }

    void load(string filename, shared_ptr<istream> content, bool create, bool replace);

    void store();
    void store(string filename);

    string serialize();

    string trimLeft(const string &str, const string &whitespace = " \t");
    static string trim(const string &str, const string &whitespace = " \t");
    shared_ptr<vector<string>> getParams(char seperator, const char *path, va_list ap);

    shared_ptr<vector<string>> getSections(const char *path, ...);
    shared_ptr<vector<string>> tryGetSections(string d, const char *path, ...);
    shared_ptr<vector<string>> getNames(const char *path, ...);
    shared_ptr<vector<string>> tryGetNames(string d, const char *path, ...);

    template <typename T>
    T get(const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);
        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0)
        {
            string errMsg = "SC-Conf: " + pathNotFound(params.get());
            cerr << errMsg << endl;
            throw new runtime_error(errMsg);
        }
        else
        {
            return convert<T>(nodes[0]->getValue());
        }
    }

    template <typename T>
    vector<T> getList(const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);
        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0)
        {
            string errMsg = "SC-Conf: " + pathNotFound(params.get());
            cerr << errMsg << endl;
            throw new runtime_error(errMsg);
        }
        else
        {
            return convertList<T>(nodes[0]->getValue());
        }
    }

    template <typename T>
    shared_ptr<vector<T>> getAll(const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);
        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0)
        {
            string errMsg = "SC-Conf: " + pathNotFound(params.get());
            cerr << errMsg << endl;
            throw new runtime_error(errMsg);
        }
        else
        {
            shared_ptr<vector<T>> result(new vector<T>());

            for (int i = 0; i < nodes.size(); i++)
            {
                result->push_back(convert<T>(nodes[i]->getValue()));
            }

            return result;
        }
    }

    template <typename T>
    T tryGet(T d, const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);

        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0)
        {
            return d;
        }

        return convert<T>(nodes[0]->getValue());
    }

    template <typename T>
    shared_ptr<vector<T>> tryGetAll(T d, const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);

        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        shared_ptr<vector<T>> result(new vector<T>());

        if (nodes.size() == 0)
        {

            result->push_back(d);

            return result;
        }

        for (int i = 0; i < nodes.size(); i++)
        {
            result->push_back(convert<T>(nodes[i]->getValue()));
        }

        return result;
    }

    template <typename T>
    void set(T value, const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);

        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        for (int i = 0; i < nodes.size(); i++)
        {
            if (nodes[i]->getType() == ConfigNode::Leaf)
            {
                nodes[i]->setValue(value);
            }
        }
    }

    /**
     * This method creates the configuration parameter if it not already exists
     */
    template <typename T>
    void setCreateIfNotExistent(T value, const char *path, ...)
    {
        va_list ap;
        va_start(ap, path);
        shared_ptr<vector<string>> params = getParams('.', path, ap);
        va_end(ap);

        vector<ConfigNode *> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);
        if (nodes.size() == 0)
        {
            if (params.get()[0].size() > 0)
            {
                vector<string> newParams;
                for (int i = 0; i < params.get()[0].size(); i++)
                {
                    nodes.clear();
                    if (newParams.size() == 0)
                    {
                        collect(this->configRoot.get(), params.get(), 0, &nodes);
                        if (nodes.size() > 0)
                        {
                            newParams.push_back(params.get()[0].at(i));
                        }
                        else
                        {
                            break;
                        }
                    }
                    else
                    {
                        newParams.push_back(params.get()[0].at(i));
                        collect(this->configRoot.get(), &newParams, 0, &nodes);
                        cout << "Size nodes: " << nodes.size() << "iteration:" << i << endl;
                        if (nodes.size() > 0)
                        {
                            newParams.push_back(params.get()[0].at(i));
                        }
                        else
                        {
                            newParams.pop_back();
                            collect(this->configRoot.get(), &newParams, 0, &nodes);

                            cout << " FINAL Size nodes: " << nodes.size() << "iteration:" << i << endl;
                            break;
                        }
                    }
                }

                vector<string> newSubList(params.get()[0].begin() + newParams.size(), params.get()[0].end());
                ConfigNode *currentNode = NULL;
                for (string newNode : newSubList)
                {
                    if (currentNode == NULL)
                    {
                        currentNode = nodes[0]->create(newNode);
                    }
                    else if (newNode.compare(newSubList.back()) != 0)
                    {
                        currentNode = currentNode->create(newNode);
                    }
                    else
                    {
                        currentNode = currentNode->create(newNode, value);
                    }
                }
            }
        }

        for (int i = 0; i < nodes.size(); i++)
        {
            if (nodes[i]->getType() == ConfigNode::Leaf)
            {
                nodes[i]->setValue(value);
            }
        }
    }
};

template <>
inline short Configuration::convert<short>(string value)
{
    return stoi(value);
}

template <>
inline unsigned short Configuration::convert<unsigned short>(string value)
{
    return stoul(value);
}

template <>
inline int Configuration::convert<int>(string value)
{
    return stoi(value);
}

template <>
inline unsigned int Configuration::convert<unsigned int>(string value)
{
    return stoul(value);
}

template <>
inline long Configuration::convert<long>(string value)
{
    return stol(value);
}

template <>
inline long double Configuration::convert<long double>(string value)
{
    return stold(value);
}

template <>
inline long long Configuration::convert<long long>(string value)
{
    return stoll(value);
}

template <>
inline unsigned long Configuration::convert<unsigned long>(string value)
{
    return stoul(value);
}

template <>
inline unsigned long long Configuration::convert<unsigned long long>(string value)
{
    return stoull(value);
}

template <>
inline float Configuration::convert<float>(string value)
{
    return stof(value);
}

template <>
inline double Configuration::convert<double>(string value)
{
    return stod(value);
}

template <>
inline string Configuration::convert<string>(string value)
{
    return value;
}

template <>
inline bool Configuration::convert<bool>(string value)
{
    if ("false" == value || value == "False" || value == "0" || value == "FALSE")
    {
        return false;
    }
    else if ("true" == value || value == "True" || value == "1" || value == "TRUE")
    {
        return true;
    }
    string errMsg = "Configuration: unable to parse boolean. Value is: " + value;
    cerr << errMsg << endl;
    throw new runtime_error(errMsg);
}

template <>
inline vector<int> Configuration::convertList<int>(string value)
{
    std::istringstream ss(value);
    std::string listItem;
    vector<int> itemVector;
    while (std::getline(ss, listItem, LIST_ELEMENT_SEPERATOR))
    {
        itemVector.push_back(stoi(trim(listItem, " ")));
    }
    return itemVector;
}

template <>
inline vector<string> Configuration::convertList<string>(string value)
{
    std::istringstream ss(value);
    std::string listItem;
    vector<string> itemVector;
    while (std::getline(ss, listItem, LIST_ELEMENT_SEPERATOR))
    {
        itemVector.push_back(trim(listItem, " "));
    }
    return itemVector;
}
}
#endif /* CONFIGURATION_H_ */
