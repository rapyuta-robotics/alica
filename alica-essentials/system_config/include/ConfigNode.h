/*
 * ConfigNode.h
 *
 *  Created on: May 30, 2014
 *      Author: emmeda
 */

#ifndef CONFIGNODE_H_
#define CONFIGNODE_H_

using namespace std;

#include <cstddef>

namespace supplementary
{
	class ConfigNode;

	typedef shared_ptr<ConfigNode> ConfigNodePtr;

	class ConfigNode
	{

	public:

		typedef enum
		{
			Node = 0, Leaf = 1, Comment = 2,
		} Type;

	protected:

		string name;
		string value;
		ConfigNode *parent;
		vector<ConfigNodePtr> children;
		int depth;
		Type type;

	public:

		ConfigNode(string name) :
				name(name), value(), parent(NULL), children(), depth(0), type(Node)
		{
		}

		ConfigNode(Type type, string name) :
				name(name), value(), parent(NULL), children(), depth(0), type(type)
		{
		}

		ConfigNode(string name, string &value) :
				name(name), value(value), parent(NULL), children(), depth(0), type(Leaf)
		{
		}

		ConfigNode(const ConfigNode &other) :
				name(other.name), value(other.value), parent(other.parent), children(other.children), depth(
						other.depth), type(other.type)
		{
		}

		~ConfigNode()
		{
			//				cout << "deleting " << this->name << endl;
		}

		ConfigNode *create(string name)
		{
			this->children.push_back(ConfigNodePtr(new ConfigNode(name)));
			this->children.back()->setParent(this);
			return this->children.back().get();
		}

		ConfigNode *create(Type type, string name)
		{
			this->children.push_back(ConfigNodePtr(new ConfigNode(type, name)));
			this->children.back()->setParent(this);
			return this->children.back().get();
		}

		ConfigNode *create(string name, string &value)
		{
			this->children.push_back(ConfigNodePtr(new ConfigNode(name, value)));
			this->children.back()->setParent(this);
			return this->children.back().get();
		}

		vector<ConfigNodePtr> *getChildren()
		{
			return &this->children;
		}

		ConfigNode *getParent()
		{
			return this->parent;
		}

		void setParent(ConfigNode *parent)
		{
			this->parent = parent;
			this->depth = parent->depth + 1;
		}

		const string &getValue() const
		{
			return this->value;
		}

		void setValue(string &value)
		{
			this->value = value;
		}

		const string &getName() const
		{
			return this->name;
		}

		int getDepth() const
		{
			return this->depth;
		}

		Type getType() const
		{
			return this->type;
		}

		ConfigNode &operator=(const ConfigNode &other)
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
} /* namespace ConfigNode.h */
#endif /* CONFIGNODE_H_ */
