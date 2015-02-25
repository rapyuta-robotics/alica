#include "Configuration.h"

namespace supplementary
{
	Configuration::Configuration() :
			filename(), configRoot(new ConfigNode("root"))
	{
	}

	Configuration::Configuration(string filename) :
			filename(filename), configRoot(new ConfigNode("root"))
	{
		load(filename);
	}

	Configuration::Configuration(string filename, const string content) :
			filename(filename), configRoot(new ConfigNode("root"))
	{
		load(filename, shared_ptr<istream>(new istringstream(content)), false, false);
	}

	void Configuration::load(string filename, shared_ptr<istream> content, bool, bool)
	{

		this->filename = filename;

		int linePos = 0;
		int chrPos = 0;

		string line;

		ConfigNode *currentNode = this->configRoot.get();

		while (content->good())
		{

			getline(*content, line);
			line = Configuration::trimLeft(line);

			int lineLen = line.size();

			chrPos = 1;

			linePos++;

			while (chrPos < lineLen - 1)
			{

				if (line.size() == 0)
					break;

				switch (line[0])
				{

					case '#':
					{
						string comment = line.substr(1, line.size() - 1);

						comment = Configuration::trim(comment);
						currentNode->create(ConfigNode::Comment, comment);

						chrPos += line.size() - 1;
					}
						continue;

					case '<':
					case '[':
					{
						size_t end = line.find(']');

						if (end == string::npos)
						{
							end = line.find('>');
						}

						if ((line.size() < 2) || (end == string::npos))
						{

							cerr << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
									<< ": malformed tag!" << endl;
							throw new exception();
						}

						if (end - 1 == 0)
						{
							cerr << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
									<< ": malformed tag, tag name empty!" << endl;
							throw new exception();
						}

						string name = line.substr(1, end - 1);

						if ((name[0] == '/') || (name[0] == '!'))
						{

							if (currentNode == NULL)
							{
								cerr << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
										<< ": no opening tag found!" << endl;
								throw new exception();
							}

							if (name.compare(1, name.size() - 1, currentNode->getName()) != 0)
							{
								cerr << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
										<< ": closing tag does not match opening tag!" << endl;
								throw new exception();
							}

							currentNode = currentNode->getParent();
						}
						else
						{
							ConfigNode *x = currentNode->create(name);
							currentNode = x;
						}

						if (end < line.size() - 1)
						{
							line = line.substr(end + 1, line.size() - end - 1);
						}

						chrPos += (end + 1);
					}
						break;

					default:
						chrPos++;

						if ((line[0] != ' ') && (line[0] != '\t'))
						{

							size_t curPos = 0;
							bool inString = false;

							ostringstream ss;

							while (curPos < line.size())
							{
								if (line[curPos] == '"')
								{
									inString = !inString;
									curPos++;
								}

								if (curPos < line.size())
								{

									ss << line[curPos];
									curPos++;
								}
							}

							line = (curPos >= line.size() - 1 ? "" : line.substr(curPos + 1, line.size() - curPos - 1));

							chrPos += (curPos - 1);

							string element = ss.str();
							string key;
							string value;

							size_t eq = element.find('=');

							if (eq != string::npos)
							{
								key = element.substr(0, eq - 1);
								value = element.substr(eq + 1, element.size() - eq - 1);

								key = Configuration::trim(key);
								value = Configuration::trim(value);
							}

							currentNode->create(key, value);
						}
						else
						{
							line = line.substr(1, line.size() - 1);
						}

						break;
				}
			}
		}

		if (this->configRoot.get() != currentNode)
		{
			cout << "Parse error in " << filename << ", line " << linePos << " character " << line.size()
					<< ": no closing tag found!" << endl;
			throw new exception();
		}
	}

	void Configuration::serialize_internal(ostringstream *ss, ConfigNode *node)
	{

		if (node == NULL)
			return;

		if (node->getType() == ConfigNode::Node)
		{

			*ss << string(node->getDepth(), '\t') << "[" << node->getName() << "]" << endl;

			for (vector<ConfigNodePtr>::iterator itr = node->getChildren()->begin(); itr != node->getChildren()->end();
					itr++)
			{
				serialize_internal(ss, (*itr).get());
			}

			*ss << string(node->getDepth(), '\t') << "[!" << node->getName() << "]" << endl;

		}
		else if (node->getType() == ConfigNode::Leaf)
		{

			*ss << string(node->getDepth(), '\t') << node->getName() << " = " << node->getValue() << endl;

		}
		else
		{ // Comment

			*ss << string(node->getDepth(), '\t') << "# " << node->getName() << endl;

		}
	}

	void Configuration::store()
	{
		if (this->filename.size() > 0)
		{
			store(this->filename);
		}
	}

	void Configuration::store(string filename)
	{
		ostringstream ss;
		ofstream os(filename.c_str(), ios_base::out);

		serialize_internal(&ss, this->configRoot.get());

		os << ss.str();
	}

	string Configuration::serialize()
	{
		ostringstream ss;
		serialize_internal(&ss, this->configRoot.get());
		return ss.str();
	}

	void Configuration::collect(ConfigNode *node, vector<string> *params, size_t offset, vector<ConfigNode *> *result)
	{

		vector<ConfigNodePtr> *children = node->getChildren();
		if (offset == params->size())
		{
			result->push_back(node);
			return;
		}
		for (size_t i = offset; i < params->size(); i++)
		{
			bool found = false;

			for (size_t j = 0; j < children->size(); j++)
			{

				if ((*children)[j]->getName().compare((*params)[i]) == 0)
				{
					collect((*children)[j].get(), params, offset + 1, result);
					found = true;
				}
			}

			if (!found)
			{
				return;
			}
		}
	}

	void Configuration::collectSections(ConfigNode *node, vector<string> *params, size_t offset,
										vector<ConfigNode *> *result)
	{

		vector<ConfigNodePtr> *children = node->getChildren();

		if (offset == params->size())
		{
			for (unsigned int i = 0; i < children->size(); i++)
			{
				result->push_back((*children)[i].get());
			}
			return;
		}

		for (size_t i = offset; i < params->size(); i++)
		{
			bool found = false;
			for (size_t j = 0; j < children->size(); j++)
			{
				if ((*children)[j]->getName().compare((*params)[i]) == 0)
				{
					collectSections((*children)[j].get(), params, offset + 1, result);
					found = true;
				}
			}
			if (!found)
				return;
		}
	}

	/**
	 * Creates a suitable error message, if the given path was not found.
	 * @param params The path which does not exist.
	 * @return The corresponding error message.
	 */
	string Configuration::pathNotFound(vector<string> *params)
	{
		ostringstream os;
		if ((params == NULL) || (params->size() == 0))
		{
			os << "Empty path not found in " << this->filename << "!" << endl;
		}
		else
		{
			os << "Configuration: Path '" << (*params)[0];
			for (size_t i = 1; i < params->size(); i++)
			{
				os << "." << (*params)[i];
			}
			os << "' not found in " << this->filename << "!" << endl;
		}
		return os.str();
	}

	/**
	 * Allows to collect the names of all sections in the given path. It does not include the keys of key-value pair in the sections.
	 * @param path
	 * @return A vector with the names of all sections in the given path.
	 */
	shared_ptr<vector<string> > Configuration::getSections(const char *path, ...)
	{
		va_list ap;
		va_start(ap, path);
		shared_ptr<vector<string> > params = getParams('.', path, ap);
		va_end(ap);

		vector<ConfigNode *> nodes;

		collectSections(this->configRoot.get(), params.get(), 0, &nodes);

		shared_ptr<vector<string> > result(new vector<string>());

		if (nodes.size() == 0)
		{
			cerr << pathNotFound(params.get()) << endl;
			throw exception();
		}

		for (unsigned int i = 0; i < nodes.size(); i++)
		{

			if (nodes[i]->getType() == ConfigNode::Node)
			{
				result->push_back(nodes[i]->getName());
			}
		}

		return result;
	}

	/**
	 * Allows to collect all keys in the given path. It does not include section names.
	 * @param path
	 * @return A vector with all keys or names of the given path.
	 */
	shared_ptr<vector<string> > Configuration::getNames(const char *path, ...)
	{
		va_list ap;
		va_start(ap, path);
		shared_ptr<vector<string> > params = getParams('.', path, ap);
		va_end(ap);

		vector<ConfigNode *> nodes;

		collectSections(this->configRoot.get(), params.get(), 0, &nodes);

		shared_ptr<vector<string> > result(new vector<string>());

		if (nodes.size() == 0)
		{
			cerr << pathNotFound(params.get()) << endl;
			throw exception();
		}

		for (size_t i = 0; i < nodes.size(); i++)
		{
			if (nodes[i]->getType() == ConfigNode::Leaf)
			{
				result->push_back(nodes[i]->getName());
			}
		}
		return result;
	}

	shared_ptr<vector<string> > Configuration::tryGetSections(string d, const char *path, ...)
	{
		va_list ap;
		va_start(ap, path);
		shared_ptr<vector<string> > params = getParams('.', path, ap);
		va_end(ap);

		vector<ConfigNode *> nodes;

		collect(this->configRoot.get(), params.get(), 0, &nodes);

		shared_ptr<vector<string> > result(new vector<string>());

		if (nodes.size() == 0)
		{
			result->push_back(d);
			return result;
		}

		for (size_t i = 0; i < nodes.size(); i++)
		{
			if (nodes[i]->getType() == ConfigNode::Node)
			{
				result->push_back(nodes[i]->getName());
			}
		}

		return result;
	}

	/**
	 * Collects the names of all child sections at the given level.
	 * @param d the default value which should be returned if the given path does not exist.
	 * @param path Specifies the level inside the configuration, at which the names of the child sections should be collected.
	 * @return The names of the sections at the given path
	 */
	shared_ptr<vector<string> > Configuration::tryGetNames(string d, const char *path, ...)
	{
		va_list ap;
		va_start(ap, path);
		shared_ptr<vector<string> > params = getParams('.', path, ap);
		va_end(ap);

		vector<ConfigNode *> nodes;

		collect(this->configRoot.get(), params.get(), 0, &nodes);

		shared_ptr<vector<string> > result(new vector<string>());

		if (nodes.size() == 0)
		{
			result->push_back(d);
			return result;
		}

		for (size_t i = 0; i < nodes.size(); i++)
		{
			if (nodes[i]->getType() == ConfigNode::Leaf)
			{
				result->push_back(nodes[i]->getName());
			}
		}
		return result;
	}

	/**
	 * Removes the given whitespaces at the beginning of the string.
	 * @param str The string which should be trimmed.
	 * @param whitespace The whitespaces which should be removed.
	 * @return The trimmed string.
	 */
	string Configuration::trimLeft(const string& str, const string& whitespace)
	{
		const auto strBegin = str.find_first_not_of(whitespace);
		if (strBegin == string::npos)
		{
			return ""; // no content
		}
		return str.substr(strBegin, str.length());
	}

	/**
	 * Removes the given whitespaces at the beginning and the end of the string.
	 * @param str The string which should be trimmed.
	 * @param whitespace The whitespaces which should be removed.
	 * @return The trimmed string.
	 */
	string Configuration::trim(const string& str, const string& whitespace)
	{
		const auto strBegin = str.find_first_not_of(whitespace);
		if (strBegin == string::npos)
		{
			return ""; // no content
		}
		const auto strEnd = str.find_last_not_of(whitespace);
		const auto strRange = strEnd - strBegin + 1;

		return str.substr(strBegin, strRange);
	}

	/**
	 * Splits the given strings if it finds the given seperator.
	 * @param seperator
	 * @param path
	 * @param ap
	 * @return The list of strings after everything was splitted.
	 */
	shared_ptr<vector<string> > Configuration::getParams(char seperator, const char *path, va_list ap)
	{
		shared_ptr<vector<string> > params = make_shared <vector<string> >();
		if (path != NULL)
		{
			const char *temp = path;
			do
			{
				string::size_type p = 0;
				string::size_type q;
				string charString = temp;
				while ((q = charString.find(seperator, p)) != string::npos)
				{
//					cout << "SC-Conf: Adding-InLoop: '" << string(temp, p, q-p) << "'" << endl;
					params->emplace_back(temp, p, q - p);
					p = q + 1;
				}
//				cout << "SC-Conf: Adding-AfterLoop: '" << string(temp, p, charString.length()-p) << "'" << endl;
				params->emplace_back(temp, p, charString.length() - p);
			} while ((temp = va_arg(ap, const char *)) != NULL);
		}
		return params;
	}
}
