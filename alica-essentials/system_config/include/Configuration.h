#pragma once

#include <algorithm>
#include <cstdarg>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <vector>
//#include "boost/lexical_cast.hpp"

#include "ConfigNode.h"

namespace essentials
{
class Configuration
{
protected:
    static const char LIST_ELEMENT_SEPERATOR = ',';
    std::string filename;
    void collect(ConfigNode* node, std::vector<std::string>* params, size_t offset, std::vector<ConfigNode*>* result);
    void collectSections(ConfigNode* node, std::vector<std::string>* params, size_t offset, std::vector<ConfigNode*>* result);
    std::string pathNotFound(std::vector<std::string>* params);

    ConfigNodePtr configRoot;

    void serialize_internal(std::ostringstream* ss, ConfigNode* node);
    void serialize_without_root(std::ostringstream* ss, ConfigNode* node);

    template <typename Target>
    Target convert(std::string value)
    {
        std::string errMsg = "Configuration: Type not handled! Value to be converted was: " + value;
        std::cerr << errMsg << std::endl;
        throw std::runtime_error(errMsg);
    }

    template <typename Target>
    std::vector<Target> convertList(std::string value)
    {
        std::string errMsg = "Configuration: List Type not handled! Value to be converted was: " + value;
        std::cerr << errMsg << std::endl;
        throw std::runtime_error(errMsg);
    }

public:
    Configuration();
    Configuration(std::string filename);
    Configuration(std::string filename, const std::string content);

    inline void load(std::string filename)
    {
        load(filename, std::shared_ptr<std::ifstream>(new std::ifstream(filename.c_str(), std::ifstream::in)), false, false);
    }

    void load(std::string filename, std::shared_ptr<std::istream> content, bool create, bool replace);

    void store();
    void store(std::string filename);

    std::string serialize();

    std::string trimLeft(const std::string& str, const std::string& whitespace = " \t");
    static std::string trim(const std::string& str, const std::string& whitespace = " \t");
    std::shared_ptr<std::vector<std::string>> getParams(char seperator, const char* path, va_list ap);

    std::shared_ptr<std::vector<std::string>> getSections(const char* path, ...);
    std::shared_ptr<std::vector<std::string>> tryGetSections(std::string d, const char* path, ...);
    std::shared_ptr<std::vector<std::string>> getNames(const char* path, ...);
    std::shared_ptr<std::vector<std::string>> tryGetNames(std::string d, const char* path, ...);

    template <typename T>
    T get(const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);
        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0) {
            std::string errMsg = "SC-Conf: " + pathNotFound(params.get());
            std::cerr << errMsg << std::endl;
            throw std::runtime_error(errMsg);
        } else {
            return convert<T>(nodes[0]->getValue());
        }
    }

    template <typename T>
    std::vector<T> getList(const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);
        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0) {
            std::string errMsg = "SC-Conf: " + pathNotFound(params.get());
            std::cerr << errMsg << std::endl;
            throw std::runtime_error(errMsg);
        } else {
            return convertList<T>(nodes[0]->getValue());
        }
    }

    template <typename T>
    std::shared_ptr<std::vector<T>> getAll(const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);
        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0) {
            std::string errMsg = "SC-Conf: " + pathNotFound(params.get());
            std::cerr << errMsg << std::endl;
            throw std::runtime_error(errMsg);
        } else {
            std::shared_ptr<std::vector<T>> result(new std::vector<T>());

            for (int i = 0; i < nodes.size(); i++) {
                result->push_back(convert<T>(nodes[i]->getValue()));
            }

            return result;
        }
    }

    template <typename T>
    T tryGet(T d, const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);

        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        if (nodes.size() == 0) {
            return d;
        }

        return convert<T>(nodes[0]->getValue());
    }

    template <typename T>
    std::shared_ptr<std::vector<T>> tryGetAll(T d, const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);

        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        std::shared_ptr<std::vector<T>> result(new std::vector<T>());

        if (nodes.size() == 0) {

            result->push_back(d);

            return result;
        }

        for (int i = 0; i < nodes.size(); i++) {
            result->push_back(convert<T>(nodes[i]->getValue()));
        }

        return result;
    }

    template <typename T>
    void set(T value, const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);

        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);

        for (int i = 0; i < nodes.size(); i++) {
            if (nodes[i]->getType() == ConfigNode::Leaf) {
                nodes[i]->setValue(value);
            }
        }
    }

    /**
     * This method creates the configuration parameter if it not already exists
     */
    template <typename T>
    void setCreateIfNotExistent(T value, const char* path, ...)
    {
        va_list ap;
        va_start(ap, path);
        std::shared_ptr<std::vector<std::string>> params = getParams('.', path, ap);
        va_end(ap);

        std::vector<ConfigNode*> nodes;

        collect(this->configRoot.get(), params.get(), 0, &nodes);
        if (nodes.size() == 0) {
            if (params.get()[0].size() > 0) {
                std::vector<std::string> newParams;
                for (int i = 0; i < params.get()[0].size(); i++) {
                    nodes.clear();
                    if (newParams.size() == 0) {
                        collect(this->configRoot.get(), params.get(), 0, &nodes);
                        if (nodes.size() > 0) {
                            newParams.push_back(params.get()[0].at(i));
                        } else {
                            break;
                        }
                    } else {
                        newParams.push_back(params.get()[0].at(i));
                        collect(this->configRoot.get(), &newParams, 0, &nodes);
                        std::cout << "Size nodes: " << nodes.size() << "iteration:" << i << std::endl;
                        if (nodes.size() > 0) {
                            newParams.push_back(params.get()[0].at(i));
                        } else {
                            newParams.pop_back();
                            collect(this->configRoot.get(), &newParams, 0, &nodes);

                            std::cout << " FINAL Size nodes: " << nodes.size() << "iteration:" << i << std::endl;
                            break;
                        }
                    }
                }

                std::vector<std::string> newSubList(params.get()[0].begin() + newParams.size(), params.get()[0].end());
                ConfigNode* currentNode = NULL;
                for (const std::string& newNode : newSubList) {
                    if (currentNode == NULL) {
                        currentNode = nodes[0]->create(newNode);
                    } else if (newNode.compare(newSubList.back()) != 0) {
                        currentNode = currentNode->create(newNode);
                    } else {
                        currentNode = currentNode->create(newNode, value);
                    }
                }
            }
        }

        for (int i = 0; i < nodes.size(); i++) {
            if (nodes[i]->getType() == ConfigNode::Leaf) {
                nodes[i]->setValue(value);
            }
        }
    }
};

template <>
inline short Configuration::convert<short>(std::string value)
{
    return stoi(value);
}

template <>
inline unsigned short Configuration::convert<unsigned short>(std::string value)
{
    return stoul(value);
}

template <>
inline int Configuration::convert<int>(std::string value)
{
    return stoi(value);
}

template <>
inline unsigned int Configuration::convert<unsigned int>(std::string value)
{
    return stoul(value);
}

template <>
inline long Configuration::convert<long>(std::string value)
{
    return stol(value);
}

template <>
inline long double Configuration::convert<long double>(std::string value)
{
    return stold(value);
}

template <>
inline long long Configuration::convert<long long>(std::string value)
{
    return stoll(value);
}

template <>
inline unsigned long Configuration::convert<unsigned long>(std::string value)
{
    return stoul(value);
}

template <>
inline unsigned long long Configuration::convert<unsigned long long>(std::string value)
{
    return stoull(value);
}

template <>
inline float Configuration::convert<float>(std::string value)
{
    return stof(value);
}

template <>
inline double Configuration::convert<double>(std::string value)
{
    return stod(value);
}

template <>
inline std::string Configuration::convert<std::string>(std::string value)
{
    return value;
}

template <>
inline bool Configuration::convert<bool>(std::string value)
{
    if ("false" == value || value == "False" || value == "0" || value == "FALSE") {
        return false;
    } else if ("true" == value || value == "True" || value == "1" || value == "TRUE") {
        return true;
    }
    std::string errMsg = "Configuration: unable to parse boolean. Value is: " + value;
    std::cerr << errMsg << std::endl;
    throw std::runtime_error(errMsg);
}

template <>
inline std::vector<int> Configuration::convertList<int>(std::string value)
{
    std::istringstream ss(value);
    std::string listItem;
    std::vector<int> itemVector;
    while (std::getline(ss, listItem, LIST_ELEMENT_SEPERATOR)) {
        itemVector.push_back(stoi(trim(listItem, " ")));
    }
    return itemVector;
}

template <>
inline std::vector<std::string> Configuration::convertList<std::string>(std::string value)
{
    std::istringstream ss(value);
    std::string listItem;
    std::vector<std::string> itemVector;
    while (std::getline(ss, listItem, LIST_ELEMENT_SEPERATOR)) {
        itemVector.push_back(trim(listItem, " "));
    }
    return itemVector;
}
} // namespace supplementary
