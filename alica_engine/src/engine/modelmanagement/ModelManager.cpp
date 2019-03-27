#include "engine/parser/ModelManager.h"

#include "engine/AlicaEngine.h"

#include <yaml-cpp/yaml.h>

namespace alica
{
void Parser::parseFile(const std::string& currentFile, const std::string& type)
{
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(currentFile);
    } catch (YAML::BadFile badFile) {
        AlicaEngine::abort("PP: parseTaskFile doc.ErrorCode: ", badFile.msg);
    }

    this->mf->createTasks(doc);
}
} // namespace alica