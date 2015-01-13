/*
 * ClingWrapper.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: Stephan Opfer
 */

#include "ClingWrapper.h"

#include "BaseLiteral.h"
#include "BoolLiteral.h"
#include "External.h"

#include "clasp/solver.h"

using namespace Clasp;
using namespace Clasp::Cli;

namespace supplementary
{

	ClingWrapper::ClingWrapper()
	{
		setMode(Mode::mode_clingo);
		this->setVerbose(0);
		this->claspAppOpts_.quiet[0] = 2;
		this->claspAppOpts_.quiet[1] = 2;
		this->claspAppOpts_.quiet[2] = 2;
		this->grOpts_.lparseDebug = Gringo::Output::LparseDebug::NONE;

	}

	ClingWrapper::~ClingWrapper()
	{
		// TODO Auto-generated destructor stub
	}

	void ClingWrapper::init()
	{
		this->setup();
		bool incremental = mode_ != mode_clasp;
		Clasp::ProblemType pt = getProblemType();
		ProgramBuilder* prg = &clasp_->start(claspConfig_, pt, incremental);
		if (incremental)
		{
			grOpts_.verbose = verbose() == UINT_MAX;
			Asp::LogicProgram* lp = mode_ != mode_gringo ? static_cast<Asp::LogicProgram*>(prg) : 0;
			grd = Gringo::make_unique<Grounder>();
			grd->parse(claspAppOpts_.input, grOpts_, lp, this); // <- hier dafür sorgen, dass die clasp output classe ersetzt wird
			this->ground("base", {});
		}
	}

	void ClingWrapper::setMode(Mode mode)
	{
		this->mode_ = mode;
	}

	Gringo::SolveResult ClingWrapper::solve()
	{
		lastModel = nullptr;
		lastSolver = nullptr;
		Gringo::SolveResult result = ClingoApp::solve(nullptr);
		return result;
	}

	void ClingWrapper::ground(std::string const &name, Gringo::FWValVec args)
	{
		ClingoApp::ground(name, args);
	}

	void ClingWrapper::addKnowledgeFile(std::string path)
	{
		this->claspAppOpts_.input.push_back(path);
	}

	bool ClingWrapper::onModel(const Clasp::Solver& s, const Clasp::Model& m)
	{
		lastModel = &m;
		lastSolver = &s;

		for (auto baseLit : this->baseLiterals)
                {
		  if (baseLit->getUpdateType() == LiteralUpdateType::PUSH)
                    baseLit->updateOnModel(&s, &m);
                }

		return true;
	}


        int ClingWrapper::getNumWildcards(Gringo::Value &value)
        {
          if (value.type() != Gringo::Value::Type::FUNC)
             return 0;

          int count = 0;

          for (uint i = 0; i < value.args().size(); ++i)
          {
            Gringo::Value arg = value.args()[i];

            if (arg.type() == Gringo::Value::Type::STRING && arg.name() == "?")
              ++count;
          }

          return count;
        }

	void ClingWrapper::assignExternal(shared_ptr<Gringo::Value> ext, bool val)
	{
		ClingoApp::assignExternal(*ext, val);
	}

	shared_ptr<Gringo::Value>ClingWrapper::ClingWrapper::assignExternal(const std::string& name, Gringo::FWValVec args, bool val)
	{
		shared_ptr<Gringo::Value> gv = (args.size() == 0 ? make_shared<Gringo::Value>(name) : make_shared<Gringo::Value>(name, args));
		assignExternal(gv, val);
		return gv;
	}

	void ClingWrapper::releaseExternal(shared_ptr<Gringo::Value> ext)
	{
		ClingoApp::releaseExternal(*ext);
	}

	shared_ptr<Gringo::Value> ClingWrapper::releaseExternal(std::string const &name, Gringo::FWValVec args)
	{
		shared_ptr<Gringo::Value> gv = (args.size() == 0 ? make_shared<Gringo::Value>(name) : make_shared<Gringo::Value>(name, args));
		releaseExternal (gv);
		return gv;
	}

	void ClingWrapper::add(const std::string& name, const Gringo::FWStringVec& params, const std::string& part)
	{
		ClingoApp::add(name, params, part);
	}

	void ClingWrapper::printLastModel(bool verbose)
	{
	        std::cout << "Last Model ";

		if (!lastModel)
		{
			std::cout << "No model found" << std::endl;
			return;
		}

		for (auto value : this->lastSolver->symbolTable())
		{
			if (this->lastModel->isTrue(value.second.lit))
				std::cout << value.second.name.c_str() << " ";
		}

		std::cout << std::endl;

		if (verbose)
		{
		        std::cout << "Symbol table: ";
		        for (auto value : this->lastSolver->symbolTable())
                        {
                                std::cout << "(" << value.second.name.c_str() << ", " << value.first << ", " <<
                                    (this->lastModel->isTrue(value.second.lit) ? "true" : "false") << ") ";
                        }

	                std::cout << std::endl;
		}
	}

	std::shared_ptr<External> const ClingWrapper::getExternal(const char* p_value)
        {
	  Gringo::Value value = ClingWrapper::splitASPExternalString(p_value);

	  if (value.type() != Gringo::Value::Type::FUNC)
	  {
	    // TODO
	    std::cout << "Bad gringo value for external ";
	    value.print(std::cout);

	    return std::shared_ptr<External>();
	  }

	  return this->getExternal(*value.name(), value.args(), *value.name(), value.args());
        }

        std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args)
        {
          return this->getExternal(name, args, name, args);
        }

        std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args,
                                                                  bool const assign)
        {
          auto ext = this->getExternal(name, args, name, args);

          ext->assign(assign);

          return ext;
        }

        std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args,
                                                                  const std::string &ground, Gringo::FWValVec groundArgs,
                                                                  bool const assign)
        {
          auto ext = this->getExternal(name, args, ground, groundArgs);

          ext->assign(assign);

          return ext;
        }

	std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args,
	                                                          const std::string &ground, Gringo::FWValVec groundArgs)
	{
	  auto value = std::make_shared<Gringo::Value>(name, args);

	  try
	  {
	    return this->externals.at(value->hash());
	  }
	  catch (std::out_of_range &e)
	  {
	    std::lock_guard<std::mutex> guard (this->mutexExternals);

            try
            {
              return this->externals.at(value->hash());
            }
            catch (std::out_of_range &e)
            {
              External* ptr = new External(this, value);
              auto external = std::shared_ptr<External>(ptr);
              this->externals.insert(std::pair<size_t, std::shared_ptr<External>>(value->hash(), external));

              this->ground(ground, groundArgs);

              return external;
            }
	  }
	}

        bool ClingWrapper::query(std::string const &name, Gringo::FWValVec args)
        {
          if (this->lastModel == nullptr || this->lastSolver == nullptr)
            return false;

          int literalId = -1;
          Gringo::Value query(name, args);

          for (auto lit : this->existingLiterals)
          {
            const Gringo::Value& value = std::get<1>(lit);
            literalId = std::get<0>(lit);

            if (false == this->lastModel->isTrue(this->lastSolver->symbolTable()[literalId].lit))
            	continue;

            if (this->checkMatchValues(&query, &value))
            {
              return true;
            }
          }

          return false;
        }


        std::unique_ptr<std::vector<Gringo::Value>> ClingWrapper::queryAllTrue(std::string const &name, Gringo::FWValVec args)
        {
          Gringo::Value value(name, args);

          return this->queryAllTrue(&value);
        }

        std::unique_ptr<std::vector<Gringo::Value>> ClingWrapper::queryAllTrue(std::shared_ptr<Gringo::Value> query)
        {
          return this->queryAllTrue(query.get());
        }

        std::unique_ptr<std::vector<Gringo::Value>> ClingWrapper::queryAllTrue(Gringo::Value* query)
        {
          if (this->lastModel == nullptr || this->lastSolver == nullptr)
            return nullptr;

          std::unique_ptr<std::vector<Gringo::Value>> values(new std::vector<Gringo::Value>);
          int literalId;
          Gringo::Value value;

          for (auto lit : this->existingLiterals)
          {
            literalId = std::get<0>(lit);
            value = std::get<1>(lit);

            if (false == this->lastModel->isTrue(this->lastSolver->symbolTable()[literalId].lit))
                continue;

            if (this->checkMatchValues(query, &value))
            {
                values->push_back(value);
            }
          }

          return std::move(values);
        }

        void ClingWrapper::registerLiteral(unsigned int literal, Gringo::Value value)
        {
          std::lock_guard<std::mutex> guard (this->mutexLiterals);
          this->existingLiterals.push_back(std::tuple<unsigned int, Gringo::Value>(literal, value));

          for (auto baseLit : this->baseLiterals)
          {
            if (baseLit->getCheckNewLiterals())
              baseLit->check(literal, value);
          }

//          std::cout << "Symbol Muh " << literal << ", ";
//          value.print(std::cout);
//          std::cout << std::endl;
        }

        std::shared_ptr<BoolLiteral> const ClingWrapper::getBoolLiteral(std::string const &name, Gringo::FWValVec args)
        {
          std::lock_guard<std::mutex> guard (this->mutexLiterals);

          Gringo::Value query(name, args);

          for (auto lit : this->baseLiterals)
          {
            if (lit->getType() == LiteralType::BOOL && lit->getQuery() == query)
            {
                return std::static_pointer_cast<BoolLiteral>(lit);
            }
          }

          int wildcards = this->getNumWildcards(query);

          if (wildcards != 0)
          {
            throw std::exception();
          }

          auto boolLiteral = std::make_shared<BoolLiteral>(this, query);
          this->baseLiterals.push_back(boolLiteral);

          for (auto lit : this->existingLiterals)
          {
            boolLiteral->check(std::get<0>(lit), std::get<1>(lit));
          }

          return boolLiteral;
        }

        bool ClingWrapper::checkMatchValues(const Gringo::Value* value1, const Gringo::Value* value2)
        {
          if (value2->type() != Gringo::Value::Type::FUNC)
            return false;

          if (value1->name() != value2->name())
            return false;

          if (value1->args().size() != value2->args().size())
            return false;

          for (uint i = 0; i < value1->args().size(); ++i)
          {
            Gringo::Value arg = value1->args()[i];

            if (arg.type() == Gringo::Value::Type::ID && arg.name() == "?")
              continue;

            if (arg != value2->args()[i])
              return false;
          }

          return true;
        }

        const Clasp::Model* ClingWrapper::getLastModel()
        {
          return this->lastModel;
        }

        const Clasp::Solver* ClingWrapper::getLastSolver()
        {
          return this->lastSolver;
        }

        Gringo::Value ClingWrapper::splitASPExternalString(const char* p_aspString)
        {
          char *aspString = new char[std::strlen(p_aspString)+1];
          std::strcpy(aspString,p_aspString);

          Gringo::Value value = splitASPExternalString(aspString);
          delete[] aspString;

          return value;
        }

        Gringo::Value ClingWrapper::splitASPExternalString(char* p_aspString)
          {
            if (std::strlen(p_aspString) == 0)
              return Gringo::Value();

            std::vector<Gringo::Value> vec;

            char* values = std::strchr(p_aspString, '(');
            char* valuesEnd = std::strrchr(p_aspString, ')');
            char* nameEnd = values;
            char tmpEnd, tmp;
            char *index1, *index2;

            if (values == nullptr || valuesEnd == nullptr || valuesEnd < values)
            {
              return Gringo::Value(p_aspString);
            }

            tmpEnd = *valuesEnd;
            *valuesEnd = '\0';

            ++values;

            while (valuesEnd > values)
            {
//              std::cout << "##" << values << std::endl;
              index1 = std::strchr(values, '(');
              index2 = std::strchr(values, ',');

              if (index1 >= valuesEnd)
                index1 = nullptr;

              if (index2 >= valuesEnd)
                index2 = nullptr;

              if (index2 == values)
              {
                ++values;
                continue;
              }

              if (index2 != nullptr && index1 == nullptr)
              {
                tmp = *index2;
                *index2 = '\0';
                toGringoValue(values, &vec);
                *index2 = tmp;
                values = index2+1;
              }
              else if (index2 == nullptr && index1 != nullptr)
              {
                index1 = std::strrchr(values, ')') +1;
                tmp = *index1 ;
                *index1 = '\0';
                vec.push_back(splitASPExternalString(values));
                *index1 = tmp;
                values = index1+1;
              }
              else if (index2 == nullptr && index1 == nullptr)
              {
                toGringoValue(values, &vec);
                values = valuesEnd;
              }
              else if (index2 < index1)
              {
                tmp = *index2;
                *index2 = '\0';
                toGringoValue(values, &vec);
                *index2 = tmp;
                values = index2 + 1;
              }
              else
              {
                index1 = std::strrchr(values, ')')+1;
                tmp = *index1;
                *index1 = '\0';
                vec.push_back(splitASPExternalString(values));
                *index1 = tmp;
                values = index1+1;
              }
            }

            *valuesEnd = tmpEnd;

            return Gringo::Value(std::string(p_aspString, nameEnd), vec);
          }

          void ClingWrapper::toGringoValue(const char* p_string, std::vector<Gringo::Value>* vec)
          {
            char* end;

            int number =  std::strtol(p_string, &end, 10);

            if ((end == p_string) || (*end != '\0'))
            {
              vec->push_back(Gringo::Value(p_string));
            }
            else
            {
              vec->push_back(Gringo::Value(number));
            }
          }

} /* namespace supplementary */

