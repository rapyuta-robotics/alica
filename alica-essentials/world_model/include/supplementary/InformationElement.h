#pragma once

#include <memory>
#include <iostream>

namespace supplementary
{
typedef long long InfoTime;

/**
 * Information element stores one information and provides required meta
 * data. The information should not be changed.
 */
template <typename T>
class InformationElement
{
  public:
    /**
     *
     * @param information
     * @param creationTime
     * @param validityDuration
     * @param certainty
     */
    InformationElement(const T information, InfoTime creationTime, InfoTime validityDuration, double certainty)
        : information(information)
        , creationTime(creationTime)
        , validityTime(creationTime + validityDuration)
        , certainty(certainty)
    {
    }

    /**
     * Default destructor
     */
    virtual ~InformationElement()
    {
    }

    /**
     * Returns the time when the contained information was created.
     * For example, when the information entered the system or when
     * the sensor did record the data.
     * @return InfoTime time when this information was created
     */
    InfoTime getCreationTime() const
    {
        return this->creationTime;
    }

    /**
     * Returns the time until this information is considered valid. At
     * a later time this information is likely to be invalid, e.g.,
     * the objects position probably has changed.
     * @return InfoTime time until this information is considered valid
     */
    InfoTime getValidityTime() const
    {
        return this->validityTime;
    }

    /**
     * Returns the certainty about the trueness of the information,
     * when the information was created.
     * @return double certainty about the trueness of the information.
     */
    double getCertainty() const
    {
        return this->certainty;
    }

    /**
     * Returns true if the validitiyTime is not over, false otherwise.
     * @return bool true if the validityTime is not over, false otherwise.
     */
    bool isValid() const
    {
        return this->validityTime > 0; // TODO: replace 0 with currentTime;
    }

    /**
     * Returns the information stored in this container.
     * @return std::shared_ptr<T> Shared Pointer to the stored information.
     */
    const T getInformation() const
    {
        return this->information;
    }

  private:
    const T information;            /**< the stored information */
    InfoTime creationTime;          /**< time this information was created */
    InfoTime validityTime;          /**< the latest time this information is considered to be valid */
    double certainty;               /**< how certain the information was at the moment it was created */
};

} /* namespace supplementary */
