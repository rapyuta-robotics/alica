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
#include "clasp/cli/clasp_options.h"

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
	          this->claspConfig_.set(Clasp::Cli::OptionKey::option_category_solver, "opt-strategy=5");
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

        void ClingWrapper::setOptStrategie(int value)
        {
          this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_strategy, std::to_string(value).c_str());
        }

        void ClingWrapper::setHeuristic(std::string value)
        {
          this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, value.c_str());
        }

        void ClingWrapper::setParallelMode(int threadCount)
        {
          // works strange or does nothing?
          this->claspConfig_.set(Clasp::Cli::OptionKey::opt_parallel_mode, std::to_string(threadCount).c_str());
        }

        void ClingWrapper::setSaveProgress(int saveProgress)
        {
          this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, std::to_string(saveProgress).c_str());
          this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restart_on_model, "");
        }

        void ClingWrapper::setPredefConfiguration(PredefinedConfigurations config)
        {
          switch(config)
          {
            case tweety:
              //--eq=3
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_eq, "3");
              //--trans-ext=dynamic
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_trans_ext, "dynamic");
              //--del-init=1000,17526
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_init, "1000,17526");
              //--del-max=2000000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_max, "2000000");
              //--strengthen=recursive,0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_strengthen, "recursive,0");
              //--otfs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_otfs, "2");
              //--heuristic=Vsids
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, "Vsids");
              //--vsids-decay=92
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_vsids_decay, "92");
              //--init-moms
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_init_moms, "");
              //--score-other=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_score_other, "2");
              //--deletion=basic,50,0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_deletion, "basic,50,0");
              //--del-cfl=+,2000,100,20
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_cfl, "+,2000,100,20");
              //--del-grow=0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_grow, "0");
              //--del-glue=2,0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_glue, "2,0");
              //--update-lbd=1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_update_lbd, "1");
              //--del-estimate=1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_estimate, "1");
              //--save-progress=160
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, "160");
              //--init-watches=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_init_watches, "2");
              //--restarts=L,60
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restarts, "L,60");
              //--local-restarts
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_local_restarts, "");
              //--loops=shared
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_loops, "shared");
              break;

            case trendy:
              //--sat-p=20,25,240,-1,1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_sat_prepro, "20,25,240,-1,1");
              //--trans-ext=dynamic
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_trans_ext, "dynamic");
              //--heuristic=Vsids
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, "Vsids");
              //--restarts=D,100,0.7
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restarts, "D,100,0.7");
              //--deletion=basic,50,0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_deletion, "basic,50,0");
              //--del-init=3.0,500,19500
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_init, "3.0,500,19500");
              //--del-grow=1.1,20.0,x,100,1.5
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_grow, "1.1,20.0,x,100,1.5");
              //--del-cfl=+,10000,2000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_cfl, "+,10000,2000");
              //--del-glue=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_glue, "2");
              //--strengthen=recursive
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_strengthen, "recursive");
              //--update-lbd
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_update_lbd, "");
              //--otfs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_otfs, "2");
              //--save-p=75
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, "75");
              //--counter-restarts=3
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_counter_restarts, "3");
              //--counter-bump=1023
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_counter_bump, "1023");
              //--reverse-arcs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_reverse_arcs, "2");
              //--contraction=250
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_contraction, "250");
              //--loops=common
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_loops, "common");
              //--opt-heu=1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_heuristic, "1");
              //--opt-strat=5
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_strategy, "5");
              break;

            case frumpy:
              //--eq=5
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_eq, "5");
              //--heuristic=Berkmin
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, "Berkmin");
              //--restarts=x,100,1.5
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restarts, "x,100,1.5");
              //--deletion=basic,75
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_deletion, "basic,75");
              //--del-init=3.0,200,40000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_init, "3.0,200,40000");
              //--del-max=400000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_max, "400000");
              //--contraction=250
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_contraction, "250");
              //--loops=common
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_loops, "common");
              //--save-p=180
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, "180");
              //--del-grow=1.1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_grow, "1.1");
              //--strengthen=local
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_strengthen, "local");
              //--sign-def=4
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_sign_def, "4");
              break;

            case crafty:
              //--sat-p=10,25,240,-1,1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_sat_prepro, "10,25,240,-1,1");
              //--trans-ext=dynamic
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_trans_ext, "dynamic");
              //--backprop
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_backprop, "");
              //--save-p=180
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, "180");
              //--heuristic=Vsids
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, "Vsids");
              //--restarts=x,128,1.5
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restarts, "x,128,1.5");
              //--deletion=basic,75,0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_deletion, "basic,75,0");
              //--del-init=10.0,1000,9000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_init, "10.0,1000,9000");
              //--del-grow=1.1,20.0
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_grow, "1.1,20.0");
              //--del-cfl=+,10000,1000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_cfl, "+,10000,1000");
              //--del-glue=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_glue, "2");
              //--otfs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_otfs, "2");
              //--reverse-arcs=1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_reverse_arcs, "1");
              //--counter-restarts=3
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_counter_restarts, "3");
              //--contraction=250
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_contraction, "250");
              //--opt-heu=1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_heuristic, "1");
              //--opt-strat=1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_strategy, "1");

              break;
            case jumpy:
              //--sat-p=20,25,240,-1,1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_sat_prepro, "20,25,240,-1,1");
              //--trans-ext=dynamic
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_trans_ext, "dynamic");
              //--heuristic=Vsids
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, "Vsids");
              //--restarts=L,100
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restarts, "L,100");
              //--deletion=basic,75,2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_deletion, "basic,75,2");
              //--del-init=3.0,1000,20000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_init, "3.0,1000,20000");
              //--del-grow=1.1,25,x,100,1.5
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_grow, "1.1,25,x,100,1.5");
              //--del-cfl=x,10000,1.1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_cfl, "x,10000,1.1");
              //--del-glue=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_glue, "2");
              //--update-lbd=3
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_update_lbd, "3");
              //--strengthen=recursive
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_strengthen, "recursive");
              //--otfs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_otfs, "2");
              //--save-p=70
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, "70");
              //--opt-heu=3
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_heuristic, "3");
              //--opt-strat=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_opt_strategy, "2");
              break;

            case handy:
              //--sat-p=10,25,240,-1,1
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_sat_prepro, "10,25,240,-1,1");
              //--trans-ext=dynamic
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_trans_ext, "dynamic");
              //--backprop
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_backprop, "");
              //--heuristic=Vsids
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_heuristic, "Vsids");
              //--restarts=D,100,0.7
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_restarts, "D,100,0.7");
              //--deletion=sort,50,2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_deletion, "sort,50,2");
              //--del-max=200000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_max, "200000");
              //--del-init=20.0,1000,14000
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_init, "20.0,1000,14000");
              //--del-cfl=+,4000,600
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_cfl, "+,4000,600");
              //--del-glue=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_del_glue, "2");
              //--update-lbd
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_update_lbd, "");
              //--strengthen=recursive
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_strengthen, "recursive");
              //--otfs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_otfs, "2");
              //--save-p=20
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_save_progress, "20");
              //--contraction=600
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_contraction, "600");
              //--loops=distinct
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_loops, "distinct");
              //--counter-restarts=7
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_counter_restarts, "7");
              //--counter-bump=1023
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_counter_bump, "1023");
              //--reverse-arcs=2
              this->claspConfig_.set(Clasp::Cli::OptionKey::opt_reverse_arcs, "2");
              break;
          }
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

	shared_ptr<Gringo::Value> ClingWrapper::ClingWrapper::assignExternal(const std::string& name, Gringo::FWValVec args, bool val)
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
		releaseExternal(gv);
		return gv;
	}

	void ClingWrapper::add(const std::string& name, const Gringo::FWStringVec& params, const std::string& part)
	{
		ClingoApp::add(name, params, part);
	}

	void ClingWrapper::printLastModel(bool verbose)
	{
		std::cout << toStringLastModel(verbose);
	}

	string ClingWrapper::toStringLastModel(bool verbose)
	{
		stringstream ss;
		ss << "Last Model ";

		if (!lastModel)
		{
			ss << "No model found" << std::endl;
			return ss.str();
		}

		for (auto value : this->lastSolver->symbolTable())
		{
			if (this->lastModel->isTrue(value.second.lit))
				ss << value.second.name.c_str() << " ";
		}

		ss << std::endl;

		if (verbose)
		{
			ss << "Symbol table: ";
			for (auto value : this->lastSolver->symbolTable())
			{
				ss << "(" << value.second.name.c_str() << ", " << value.first << ", " << (this->lastModel->isTrue(value.second.lit) ? "true" : "false") << ") ";

	                        ss << std::endl;
			}

			ss << std::endl;
		}
		return ss.str();
	}

	std::shared_ptr<External> const ClingWrapper::getExternal(const char* p_value)
        {
	  Gringo::Value value = ClingWrapper::stringToValue(p_value);

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

	std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args, bool const assign)
	{
		auto ext = this->getExternal(name, args, name, args);

		ext->assign(assign);

		return ext;
	}

	std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args, const std::string &ground, Gringo::FWValVec groundArgs, bool const assign)
	{
		auto ext = this->getExternal(name, args, ground, groundArgs);

		ext->assign(assign);

		return ext;
	}

	std::shared_ptr<External> const ClingWrapper::getExternal(const std::string& name, Gringo::FWValVec args, const std::string &ground, Gringo::FWValVec groundArgs)
	{
		auto value = std::make_shared<Gringo::Value>(name, args);

		try
		{
			return this->externals.at(value->hash());
		}
		catch (std::out_of_range &e)
		{
			std::lock_guard<std::mutex> guard(this->mutexExternals);

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

//	bool ClingWrapper::setConst(std::string const name, Gringo::Value value)
//	{
//	  auto ret = grd->defs.defs().find(name);
////	      return ret != grd->defs.defs().end()
////	          ? std::get<2>(ret->second)->eval()
////	          : Gringo::Value();
//
//	  if (ret == grd->defs.defs().end())
//	    return false;
//
//	  //Location const &loc, FWString name, UTerm &&value, bool defaultDef
//	  this->grd->defs.add(std::get<1>(ret->second), name, TODO,std::get<0>(ret->second));
//
////	  grd->defs.defs().emplace(name,std::make_tuple(, std::get<1>(ret->second), std::get<2>(ret->second)));
//	}

	bool ClingWrapper::query(std::string const &value)
	{
	        if (this->lastModel == nullptr || this->lastSolver == nullptr)
	                return false;

	        Gringo::Value query = ClingWrapper::stringToValue(value.c_str());

	        return this->query(&query);
	}

	bool ClingWrapper::query(std::string const &name, Gringo::FWValVec args)
	{
		if (this->lastModel == nullptr || this->lastSolver == nullptr)
			return false;

		Gringo::Value query(name, args);

		return this->query(&query);
	}

        bool ClingWrapper::query(Gringo::Value* query)
        {
                if (this->lastModel == nullptr || this->lastSolver == nullptr)
                        return false;

                int literalId = -1;

                for (auto lit : this->existingLiterals)
                {
                        const Gringo::Value& value = std::get<1>(lit);
                        literalId = std::get<0>(lit);

                        if (false == this->lastModel->isTrue(this->lastSolver->symbolTable()[literalId].lit))
                                continue;

                        if (this->checkMatchValues(query, &value))
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
		std::lock_guard<std::mutex> guard(this->mutexLiterals);
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
		std::lock_guard<std::mutex> guard(this->mutexLiterals);

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

        const long long ClingWrapper::getSolvingTime()
        {
          if (this->getStats() == nullptr)
            return -1;

          auto claspFacade = this->getStats()->clasp;

          if (claspFacade == nullptr)
            return -1;

          // time in seconds
          return claspFacade->summary().solveTime * 1000;
        }

        const long long ClingWrapper::getSatTime()
        {
          if (this->getStats() == nullptr)
            return -1;

          auto claspFacade = this->getStats()->clasp;

          if (claspFacade == nullptr)
            return -1;

          // time in seconds
          return claspFacade->summary().satTime * 1000;
        }

        const long long ClingWrapper::getUnsatTime()
        {
          if (this->getStats() == nullptr)
            return -1;

          auto claspFacade = this->getStats()->clasp;

          if (claspFacade == nullptr)
            return -1;

          // time in seconds
          return claspFacade->summary().unsatTime * 1000;
        }

        const long ClingWrapper::getModelCount()
        {
          if (this->getStats() == nullptr)
            return -1;

          auto claspFacade = this->getStats()->clasp;

          if (claspFacade == nullptr)
            return -1;

          return claspFacade->summary().enumerated();
        }

        const long ClingWrapper::getSymbolTableSize()
        {
          if (this->lastSolver != nullptr)
            return this->lastSolver->symbolTable().size();

          return -1;
        }

        Gringo::Value ClingWrapper::stringToValue(const char* p_aspString)
        {
          char *aspString = new char[std::strlen(p_aspString)+1];
          std::strcpy(aspString,p_aspString);

          Gringo::Value value = stringToValue(aspString);
          delete[] aspString;

          return value;
        }

        Gringo::Value ClingWrapper::stringToValue(char* p_aspString)
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

              if (index1 == nullptr && index2 == values)
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
                int count = 0;
                ++index1;
                while (index1 != valuesEnd)
                {
                  if (*index1 == ')')
                  {
                    if (count == 0)
                      break;
                    else
                      --count;
                  }
                  else if (*index1 == '(')
                    ++count;

                  ++index1;
                }
                ++index1;
              //  index1 = std::strrchr(values, ')') +1;
                tmp = *index1 ;
                *index1 = '\0';
                vec.push_back(stringToValue(values));
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
                int count = 0;
                ++index1;
                while (index1 != valuesEnd)
                {
                  if (*index1 == ')')
                  {
                    if (count == 0)
                      break;
                    else
                      --count;
                  }
                  else if (*index1 == '(')
                    ++count;

                  ++index1;
                }
                ++index1;
                tmp = *index1;
                *index1 = '\0';
                vec.push_back(stringToValue(values));
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

