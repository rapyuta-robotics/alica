#include "Configuration.h"

Configuration::Configuration() :
filename(), configRoot(new ConfigNode("root"))
{
}

Configuration::Configuration(std::string filename) :
				filename(filename), configRoot(new ConfigNode("root"))
{
	load(filename);
}

Configuration::Configuration(std::string filename, const std::string content) :
				filename(filename), configRoot(new ConfigNode("root"))
{
	load(filename, std::shared_ptr<std::istream>(new std::istringstream(content)), false, false);
}

void Configuration::load(std::string filename, std::shared_ptr<std::istream> content, bool, bool)
{

	this->filename = filename;

	int linePos = 0;
	int chrPos = 0;

	std::string line;

	ConfigNode *currentNode = this->configRoot.get();

	while (content->good())
	{

		std::getline(*content, line);
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
					std::string comment = line.substr(1, line.size() - 1);

					comment = Configuration::trim(comment);
					currentNode->create(ConfigNode::Comment, comment);

					chrPos += line.size() - 1;
				}
				continue;

				case '<':
				case '[':
				{
					size_t end = line.find(']');

					if (end == std::string::npos)
					{
						end = line.find('>');
					}

					if ((line.size() < 2) || (end == std::string::npos))
					{
						std::ostringstream ss;
						ss << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
								<< ": malformed tag!";
						throw ConfigException(ss.str());
					}

					if (end - 1 == 0)
					{
						std::ostringstream ss;
						ss << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
								<< ": malformed tag, tag name empty!";
						throw ConfigException(ss.str());
					}

					std::string name = line.substr(1, end - 1);
					//							std::cout << "'" << line << "' '" << name << "' " << end << std::endl;

					if ((name[0] == '/') || (name[0] == '!'))
					{

						if (currentNode == NULL)
						{
							std::ostringstream ss;
							ss << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
									<< ": no opening tag found!";
							throw ConfigException(ss.str());
						}

						if (name.compare(1, name.size() - 1, currentNode->getName()) != 0)
						{
							std::ostringstream ss;
							ss << "Parse error in " << filename << ", line " << linePos << " character " << chrPos
									<< ": closing tag does not match opening tag!";
							throw ConfigException(ss.str());
						}

						currentNode = currentNode->getParent();

						//								std::cout << "<- " << name << std::endl;

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

						std::ostringstream ss;

						while (curPos < line.size())
						{

							/*								if ((!inString) &&
							 ((line[curPos] == '[') || (line[curPos] == '<')))
							 {
							 curPos--;
							 break;
							 }
							 */
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

						std::string element = ss.str();
						std::string key;
						std::string value;

						size_t eq = element.find('=');

						if (eq != std::string::npos)
						{
							key = element.substr(0, eq - 1);
							value = element.substr(eq + 1, element.size() - eq - 1);

							key = Configuration::trim(key);
							value = Configuration::trim(value);
						}

						//boost::any a(value);

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
		std::ostringstream ss;
		ss << "Parse error in " << filename << ", line " << linePos << " character " << line.size()
						<< ": no closing tag found!";
		throw ConfigException(ss.str());
	}
}

void Configuration::serialize_internal(std::ostringstream *ss, ConfigNode *node)
{

	if (node == NULL)
		return;

	if (node->getType() == ConfigNode::Node)
	{

		*ss << std::string(4 * node->getDepth(), ' ') << "[" << node->getName() << "]" << std::endl;

		for (std::vector<ConfigNodePtr>::iterator itr = node->getChildren()->begin(); itr != node->getChildren()->end();
				itr++)
		{
			serialize_internal(ss, (*itr).get());
		}

		*ss << std::string(4 * node->getDepth(), ' ') << "[!" << node->getName() << "]" << std::endl;

	}
	else if (node->getType() == ConfigNode::Leaf)
	{

		*ss << std::string(4 * node->getDepth(), ' ') << node->getName() << " = "
				<< boost::any_cast<std::string>(node->getValue()) << std::endl;

	}
	else
	{ // Comment

		*ss << std::string(4 * node->getDepth(), ' ') << "# " << node->getName() << std::endl;

	}
}

void Configuration::store()
{

	if (this->filename.size() > 0)
	{
		store(this->filename);
	}
}

void Configuration::store(std::string filename)
{

	std::ostringstream ss;
	std::ofstream os(filename.c_str(), std::ios_base::out);

	serialize_internal(&ss, this->configRoot.get());

	os << ss.str();
}

std::string Configuration::serialize()
{

	std::ostringstream ss;
	serialize_internal(&ss, this->configRoot.get());

	return ss.str();
}

void Configuration::collect(ConfigNode *node, std::vector<std::string> *params, size_t offset,
                            std::vector<ConfigNode *> *result)
{

	std::vector<ConfigNodePtr> *children = node->getChildren();

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
			return;
	}
}

void Configuration::collectSections(ConfigNode *node, std::vector<std::string> *params, size_t offset,
                                    std::vector<ConfigNode *> *result)
{

	std::vector<ConfigNodePtr> *children = node->getChildren();

	//		for(unsigned int i = 0; i < children->size(); i++){

	//			printf("Children %d %s\n", i, (*children)[i]->getName().c_str());

	//		}

	//		printf("offset %u\n", (unsigned int)offset);
	//		printf("params->size %u\n", (unsigned int)params->size());

	if (offset == params->size())
	{
		//			printf("pushed %s\n", node->getName().c_str());
		//result->push_back(node);
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
				//					printf("found true mit %s\n", (*children)[j]->getName().c_str());
				collectSections((*children)[j].get(), params, offset + 1, result);
				found = true;
			}
		}

		if (!found)
			return;
	}
}

std::string Configuration::pathNotFound(std::vector<std::string> *params)
{

	std::ostringstream os;

	if ((params == NULL) || (params->size() == 0))
	{

		os << "Empty path not found in " << this->filename << "!" << std::endl;

	}
	else
	{

		os << "Path '" << (*params)[0];

		for (size_t i = 1; i < params->size(); i++)
		{
			os << "." << (*params)[i];
		}

		os << "' not found in " << this->filename << "!" << std::endl;
	}

	return os.str();
}

std::shared_ptr<std::vector<std::string> > Configuration::getSections(const char *path, ...)
{

	std::shared_ptr<std::vector<std::string> > params;
	params = split(path, '.');

	std::vector<ConfigNode *> nodes;

	//        for(unsigned int i = 0; i < params->size(); i++){
	//			printf("Params %d %s\n", i, (*params)[i].c_str());

	//        }

	collectSections(this->configRoot.get(), params.get(), 0, &nodes);

	std::shared_ptr<std::vector<std::string> > result(new std::vector<std::string>());

	if (nodes.size() == 0)
	{
		throw ConfigException(pathNotFound(params.get()));
	}

	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		//printf("Nodes as result: %u %s\n", i, nodes[i]->getName().c_str());

		if (nodes[i]->getType() == ConfigNode::Node)
		{
			result->push_back(nodes[i]->getName());
		}
	}

	return result;
}

std::shared_ptr<std::vector<std::string> > Configuration::getNames(const char *path, ...)
{

	std::shared_ptr<std::vector<std::string> > params;
	params = split(path, '.');

	std::vector<ConfigNode *> nodes;

	collect(this->configRoot.get(), params.get(), 0, &nodes);

	std::shared_ptr<std::vector<std::string> > result(new std::vector<std::string>());

	if (nodes.size() == 0)
	{
		throw ConfigException(pathNotFound(params.get()));
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

std::shared_ptr<std::vector<std::string> > Configuration::tryGetSections(std::string d, const char *path, ...)
{

	std::shared_ptr<std::vector<std::string> > params;
	params = split(path, '.');

	std::vector<ConfigNode *> nodes;

	collect(this->configRoot.get(), params.get(), 0, &nodes);

	std::shared_ptr<std::vector<std::string> > result(new std::vector<std::string>());

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

std::shared_ptr<std::vector<std::string> > Configuration::tryGetNames(std::string d, const char *path, ...)
{

	std::shared_ptr<std::vector<std::string> > params;
	params = split(path, '.');

	std::vector<ConfigNode *> nodes;

	collect(this->configRoot.get(), params.get(), 0, &nodes);

	std::shared_ptr<std::vector<std::string> > result(new std::vector<std::string>());

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

std::string Configuration::trimLeft(const std::string& str,
                                    const std::string& whitespace)
{
	const auto strBegin = str.find_first_not_of(whitespace);
	if (strBegin == std::string::npos) {
		return ""; // no content
	}
	//const auto strEnd = str.find_last_not_of(whitespace);
	//const auto strRange = strEnd - strBegin + 1;

	return str.substr(strBegin, str.length());
}

std::string Configuration::trim(const std::string& str,
                                const std::string& whitespace)
{
	const auto strBegin = str.find_first_not_of(whitespace);
	if (strBegin == std::string::npos) {
		return ""; // no content
	}
	const auto strEnd = str.find_last_not_of(whitespace);
	const auto strRange = strEnd - strBegin + 1;

	return str.substr(strBegin, strRange);
}
std::shared_ptr<std::vector<std::string> > Configuration::split(const char *path , char seperator){
	std::shared_ptr<std::vector<std::string> > params(new std::vector<std::string>());
	if (path != NULL) {
		va_list ap;
		va_start(ap, path);
		const char *temp = path;
		do {
			std::vector<std::string> result;
			std::string::size_type p = 0;
			std::string::size_type q;
			std::string charString = path;
			while ((q = charString.find(seperator, p)) != std::string::npos) {
				result.emplace_back(path, p, q - p);
				p = q + 1;
			}
			result.emplace_back(path, p);
			for (size_t i = 0; i < result.size(); i++) {
				params->push_back(result[i]);
			}
		} while ((temp = va_arg(ap, const char *)) != NULL);
		va_end(ap);
	}
	return params;
}

