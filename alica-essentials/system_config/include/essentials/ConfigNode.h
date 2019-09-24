/*
 * ConfigNode.h
 *
 *  Created on: May 30, 2014
 *      Author: Stephan Opfer
 */

#pragma once

#include <cstddef>
#include <memory>

namespace essentials
{
class ConfigNode;

typedef std::shared_ptr<ConfigNode> ConfigNodePtr;

class ConfigNode
{
  public:
    typedef enum {
        Node = 0,
        Leaf = 1,
        Comment = 2,
    } Type;

  protected:
    std::string name;
    std::string value;
    ConfigNode* parent;
    std::vector<ConfigNodePtr> children;
    int depth;
    Type type;

  public:
    ConfigNode(const std::string& name)
        : name(name)
        , value()
        , parent(nullptr)
        , children()
        , depth(0)
        , type(Node)
    {
    }

    ConfigNode(Type type, const std::string& name)
        : name(name)
        , value()
        , parent(nullptr)
        , children()
        , depth(0)
        , type(type)
    {
    }

    ConfigNode(const std::string& name, const std::string& value)
        : name(name)
        , value(value)
        , parent(nullptr)
        , children()
        , depth(0)
        , type(Leaf)
    {
    }

    ConfigNode(const ConfigNode& other)
        : name(other.name)
        , value(other.value)
        , parent(other.parent)
        , children(other.children)
        , depth(other.depth)
        , type(other.type)
    {
    }

    ~ConfigNode() {}

    ConfigNode* create(const std::string& name)
    {
        this->children.push_back(ConfigNodePtr(new ConfigNode(name)));
        this->children.back()->setParent(this);
        return this->children.back().get();
    }

    ConfigNode* create(Type type, const std::string& name)
    {
        this->children.push_back(ConfigNodePtr(new ConfigNode(type, name)));
        this->children.back()->setParent(this);
        return this->children.back().get();
    }

    ConfigNode* create(const std::string& name, const std::string& value)
    {
        this->children.push_back(ConfigNodePtr(new ConfigNode(name, value)));
        this->children.back()->setParent(this);
        return this->children.back().get();
    }

    std::vector<ConfigNodePtr>* getChildren() { return &this->children; }

    std::vector<ConfigNodePtr> findChildren(const std::string& name) {
        std::vector<ConfigNodePtr> out;
        for (auto& child : children) {
            if (child->getName() == name) {
                out.push_back(child);
            }
        }
        return out;
    }

    ConfigNode* getParent() const { return this->parent; }

    void setParent(ConfigNode* parent)
    {
        this->parent = parent;
        this->depth = parent->depth + 1;
    }

    const std::string& getValue() const { return this->value; }

    void setValue(const std::string& value) { this->value = value; }

    const std::string& getName() const { return this->name; }

    int getDepth() const { return this->depth; }

    Type getType() const { return this->type; }

    ConfigNode& operator=(const ConfigNode& other)
    {
        this->name = other.name;
        this->value = other.value;
        this->parent = other.parent;
        this->children = other.children;
        this->depth = other.depth;
        this->type = other.type;

        return *this;
    }
};
} // namespace essentials
