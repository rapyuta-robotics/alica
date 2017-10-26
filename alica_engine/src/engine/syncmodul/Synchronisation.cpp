#include "engine/syncmodul/Synchronisation.h"
#include "engine/AlicaEngine.h"
#include "engine/model/Transition.h"
#include "engine/IAlicaClock.h"
#include "engine/model/SyncTransition.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/syncmodul/SyncRow.h"
#include "engine/syncmodul/SyncModul.h"
#include "engine/teamobserver/TeamObserver.h"

namespace alica
{

	Synchronisation::Synchronisation(AlicaEngine* ae) : myID(nullptr)
	{
		this->ae = ae;
		this->syncModul = nullptr;
		this->syncTransition = nullptr;
		this->syncStartTime = 0;
		this->readyForSync = false;
		this->lastTick = 0;
		this->runningPlan = nullptr;
		this->myRow = nullptr;
		this->lastTalkTime = 0;
		this->lastTalkData = nullptr;
	}

	Synchronisation::Synchronisation(AlicaEngine* ae, const supplementary::IAgentID* myID, SyncTransition* st, SyncModul* sm)
	{
		this->ae = ae;
		this->syncTransition = st;
		this->myID = myID;
		this->syncStartTime = ae->getIAlicaClock()->now() / 1000000UL;
		for (Transition* t : st->getInSync())
		{
			connectedTransitions.push_back(t->getId());
		}
		this->syncModul = sm;
		this->runningPlan = nullptr;
		this->myRow = nullptr;
		this->lastTalkTime = 0;
		this->readyForSync = false;
		this->lastTick = 0;
		this->lastTalkData = nullptr;
	}

	Synchronisation::~Synchronisation()
	{
		for(auto row : syncMatrix) {
			delete row;
		}
	}

	SyncTransition* Synchronisation::getSyncTransition()
	{
		return syncTransition;
	}

	void Synchronisation::setTick(unsigned long now)
	{
		this->lastTick = now;
	}

	void Synchronisation::changeOwnData(long transitionID, bool conditionHolds)
	{
#ifdef SM_MISC
		cout << "CHOD: ElapsedTime: "
				<< (AlicaEngine::getInstance()->getIAlicaClock()->now() / 1000000UL - this->syncStartTime) << endl;
#endif

		if (!conditionHolds)
		{
			//my condition does not hold => not ready for syncing
			this->readyForSync = false;
		}
		SyncData* sd = new SyncData();
		sd->robotID = this->myID;
		sd->transitionID = transitionID;
		sd->conditionHolds = conditionHolds;
		sd->ack = false;

		bool maySendTalk = true;

		{
			lock_guard<mutex> lock(syncMutex);
			if (myRow != nullptr)
			{

				if (/*sd->ack != myRow->getSyncData()->ack
						||*/ sd->conditionHolds != myRow->getSyncData()->conditionHolds
						|| *(sd->robotID) != *(myRow->getSyncData()->robotID)
						|| sd->transitionID != myRow->getSyncData()->transitionID)
				{
					//my sync row has changed
					myRow->setSyncData(sd);
					myRow->getReceivedBy().clear();
					this->readyForSync = false;
					myRow->getReceivedBy().push_back(this->myID);
				}
				else
				{
#ifdef SM_MISC
	cout << "ChangeOwnData: SendTalk==false" << endl;
#endif
					maySendTalk = false;
					delete sd;
				}
			}
			else //init my row
			{
				SyncRow* sr = new SyncRow(sd);
				sr->getReceivedBy().push_back(this->myID);
				this->myRow = sr;

				this->syncMatrix.push_back(sr);
			}
		}

#ifdef SM_MISC
		cout << endl;
		cout << "Matrix: ChangeOwnData" << endl;
		printMatrix();
#endif

		if (maySendTalk)
		{
			if (isSyncComplete())
			{
#ifdef SM_SUCCESS
				cout << "ChangedOwnData: Synctrans " << this->syncTransition->getId() << " ready" << endl;
#endif
				sendSyncReady();
				this->readyForSync = true;
			}
			else
			{
				sendTalk(sd);
			}
		}
	}

	/**
	 * resend lastTalk, test for failure on synchronisation timeout and stop sync on state change
	 */
	bool Synchronisation::isValid(unsigned long curTick)
	{
		bool stillActive = (this->lastTick + 2 >= curTick);

		if (!stillActive)
		{
			//notify others if i am part of the synchronisation already (i.e. have an own row)
			if (myRow != nullptr)
			{
				if (myRow->getSyncData() != nullptr)
				{
					myRow->getSyncData()->conditionHolds = false;

					sendTalk(myRow->getSyncData());
				}
			}

			return false;
		}

		unsigned long now = ae->getIAlicaClock()->now() / 1000000UL;

		if (this->lastTalkTime != 0) //talked already
		{

#ifdef SM_FAILURE
			cout << "TestTimeOut on Sync: " << this->syncTransition->getId() << endl;
#endif
			if ((now > this->syncTransition->getTalkTimeOut() + this->lastTalkTime) && !this->readyForSync)
			{
				if (this->myRow != nullptr)
				{
					sendTalk(this->myRow->getSyncData());
				}
			}
		}

#ifdef SM_FAILURE
		cout << "Synchronisation: TestTimeOut(): syncStarTime " << this->syncStartTime << endl;
#endif

		if (this->syncTransition->isFailOnSyncTimeOut())
		{
			if (now > this->syncTransition->getSyncTimeOut() + this->syncStartTime)
			{
#ifdef SM_FAILURE
				cout << "Synchronisation: TestTimeOut() sync failed" << endl;
#endif
				return false;
			}
		}

		return true;
	}

	bool Synchronisation::integrateSyncTalk(shared_ptr<SyncTalk> talk, unsigned long curTick)
	{
		if (this->readyForSync)
		{
			//do not integrate talks if we believe the sync is already finished
			return true;
		}

		bool isSynching = (this->lastTick + 2 >= curTick);

		if (!isSynching)
		{
			//do not accept messages (and send uneccessary ACKs) if we are not in a state for sync
			return false;
		}

#ifdef SM_MESSAGES
		cout << "Integrate synctalk in synchronisation" << endl;
		cout << "ST: ElapsedTime: " << (ae->getIAlicaClock()->now() - this->syncStartTime)
				<< endl;
#endif
		for (SyncData* sd : talk->syncData)
		{
#ifdef SM_MESSAGES
			cout << "syncdata for transID: " << sd->transitionID << endl;
#endif
			lock_guard<mutex> lock(syncMutex);
			{

				SyncRow* rowInMatrix = nullptr;
				for (SyncRow* row : this->syncMatrix)
				{
#ifdef SM_MESSAGES
					cout << "ROW SD: " << row->getSyncData()->robotID << " " << row->getSyncData()->transitionID << " "
							<< row->getSyncData()->conditionHolds << " " << row->getSyncData()->ack << endl;
					cout << "CUR SD: " << sd->robotID << " " << sd->transitionID << " " << sd->conditionHolds << " "
							<< sd->ack << endl;
#endif


					if (/*sd->ack == row->getSyncData()->ack
							&&*/ sd->conditionHolds == row->getSyncData()->conditionHolds
							&& *(sd->robotID) == *(row->getSyncData()->robotID)
							&& sd->transitionID == row->getSyncData()->transitionID)
					{
						rowInMatrix = row;
						break;
					}
				}

				//if(rowInMatrix != null)
				if (rowInMatrix == nullptr)
				{
#ifdef SM_MESSAGES
					cout << "NEW MATRIX row" << endl;
#endif
					SyncRow* newRow = new SyncRow(sd);
					newRow->getReceivedBy().push_back(talk->senderID);

					syncMatrix.push_back(newRow);
				}
				else
				{
#ifdef SM_MESSAGES
					cout << "Received by: " << talk->senderID << endl;
#endif
					rowInMatrix->getReceivedBy().push_back(talk->senderID);
				}
				if (isSyncComplete())
				{
#ifdef SM_MESSAGES
					cout << "IntegrateSyncTalk: Synctrans " << this->syncTransition->getId() << " ready" << endl;
#endif
					sendSyncReady();
					this->readyForSync = true;
				}
				else
				{
					//always reset this in case someone revokes his commitment
					this->readyForSync = false;
				}
#ifdef SM_MESSAGES
				cout << "Matrix: IntSyncTalk" << endl;
				printMatrix();
#endif

				//late acks...
				if (this->readyForSync)
				{
					if (allSyncReady())
					{
#ifdef SM_SUCCESSS
						Console.WriteLine("SyncDONE in Synchronisation (IntTalk): elapsed time: " + ((RosCS.RosSharp.Now()/1000000UL) - this.syncStartTime));
#endif
						//notify syncmodul
						this->syncModul->synchronisationDone(this->syncTransition);

					}
				}
			}
		}

		return true;
	}

	void Synchronisation::integrateSyncReady(shared_ptr<SyncReady> ready)
	{
		//every robot that has acknowleged my row needs to send me a SyncReady
		bool found = false;
		for (shared_ptr<SyncReady> sr : this->receivedSyncReadys)
		{
			if (*(sr->senderID) == *(ready->senderID))
			{
				found = true;
				break;
			}
		}

		if (!found)
		{
			this->receivedSyncReadys.push_back(ready);
		}
#ifdef SM_MESSAGES
		cout << "Matrix: IntSyncReady" << endl;
		printMatrix();
#endif

		//check if all robots are ready
		if (this->readyForSync)
		{
			if (allSyncReady())
			{
				//notify syncModul
#ifdef SM_SUCCESS
				cout << "SyncDONE in Synchronisation (IntReady): elapsed time: "
						<< (ae->getIAlicaClock()->now() / 1000000UL) - this->syncStartTime << endl;
#endif
				this->syncModul->synchronisationDone(this->syncTransition);
			}
		}
	}

	void Synchronisation::setSyncTransition(SyncTransition* syncTransition)
	{
		this->syncTransition = syncTransition;
	}

	bool Synchronisation::allSyncReady()
	{
		//test if all robots who acknowledged myRow have sent a SyncReady
		for (auto& robotID : this->myRow->getReceivedBy())
		{
			if (*robotID != *myID) //we do not necessarily need an ack from ourselves
			{
				bool foundRobot = false;
				for (shared_ptr<SyncReady> sr : this->receivedSyncReadys)
				{
					if (*(sr->senderID) == *(robotID))
					{
						foundRobot = true;
						break;
					}
				}

				if (!foundRobot) //at least one robot is missing
				{
					return false;
				}
			}
		}
		return true;
	}


	void Synchronisation::printMatrix()
	{
		cout << endl;
		cout << "Matrix:" << endl;

		lock_guard<mutex> lock(syncMutex);
		{

			for (SyncRow* row : this->syncMatrix)
			{
				cout << "Row: " << row->getSyncData()->robotID << " " << to_string(row->getSyncData()->transitionID) + " "
								+ to_string(row->getSyncData()->conditionHolds) << " " << row->getSyncData()->ack << " RecvBy: ";
				for (auto& robotID : row->getReceivedBy())
				{
					cout << robotID << ", ";
				}
				cout << endl;
			}
			cout << "ReceivedSyncreadys: ";
			for (shared_ptr<SyncReady> sr : this->receivedSyncReadys)
			{
				cout << sr->senderID << ", " << endl;;
			}
		}
		cout << endl;
	}

	void Synchronisation::sendTalk(SyncData* sd)
	{
		SyncTalk talk;
		talk.syncData.push_back(sd);
		this->lastTalkTime = ae->getIAlicaClock()->now() / 1000000UL;

#ifdef SM_MESSAGES
		cout << "Sending Talk TID: " << sd->transitionID << endl;
#endif

		this->syncModul->sendSyncTalk(talk);
	}

	void Synchronisation::sendSyncReady()
	{
		//send own row again to be sure
		sendTalk(myRow->getSyncData());

		SyncReady sr;
		sr.syncTransitionID = this->syncTransition->getId();
		this->syncModul->sendSyncReady(sr);
	}

	/**
	 * Before calling this method lock this mutex --> syncMutex (lock_guard<mutex> lock(syncMutex))
	 */
	bool Synchronisation::isSyncComplete()
	{
		//myRow needs to be acknowledged by all participants
		//every participant needs to believe in its condition
		//there must be at least one participant for every condition
		lock_guard<mutex> lock(rowOkMutex);
		this->rowsOK.clear();
		//collect participants
		for (long transID : this->connectedTransitions)
		{
			SyncRow* foundRow = nullptr;

//			Is not needed here
//			lock_guard<mutex> lock(syncMutex);
			for (SyncRow* row : this->syncMatrix)
			{
				if (row->getSyncData()->transitionID == transID && row->getSyncData()->conditionHolds)
				{
					foundRow = row;
					break;
				}
			}
			if (foundRow == nullptr) //no robot for transition
			{
				return false;
			}

			if (find(this->rowsOK.begin(), this->rowsOK.end(), foundRow) == this->rowsOK.end())
			{
				this->rowsOK.push_back(foundRow);
			}
		}
//		check for acks in own row
		for (SyncRow* row : this->rowsOK)
		{
			auto tmp = row->getSyncData()->robotID;
			if (find_if(this->myRow->getReceivedBy().begin(), this->myRow->getReceivedBy().end(),[&tmp](const supplementary::IAgentID *id) { return *tmp == *id; }
						) == this->myRow->getReceivedBy().end())
			{
				return false;
			}
		}
		return true;

	}

}
/* namespace alica */
