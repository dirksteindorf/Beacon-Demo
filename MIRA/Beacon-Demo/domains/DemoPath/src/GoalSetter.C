/*
 * Copyright (C) 2012 by
 *   MetraLabs GmbH (MLAB), GERMANY
 * and
 *   Neuroinformatics and Cognitive Robotics Labs (NICR) at TU Ilmenau, GERMANY
 * All rights reserved.
 *
 * Contact: info@mira-project.org
 *
 * Commercial Usage:
 *   Licensees holding valid commercial licenses may use this file in
 *   accordance with the commercial license agreement provided with the
 *   software or, alternatively, in accordance with the terms contained in
 *   a written agreement between you and MLAB or NICR.
 *
 * GNU General Public License Usage:
 *   Alternatively, this file may be used under the terms of the GNU
 *   General Public License version 3.0 as published by the Free Software
 *   Foundation and appearing in the file LICENSE.GPL3 included in the
 *   packaging of this file. Please review the following information to
 *   ensure the GNU General Public License version 3.0 requirements will be
 *   met: http://www.gnu.org/copyleft/gpl.html.
 *   Alternatively you may (at your option) use any later version of the GNU
 *   General Public License if such license has been publicly approved by
 *   MLAB and NICR (or its successors, if any).
 *
 * IN NO EVENT SHALL "MLAB" OR "NICR" BE LIABLE TO ANY PARTY FOR DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF "MLAB" OR
 * "NICR" HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * "MLAB" AND "NICR" SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND "MLAB" AND "NICR" HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS OR MODIFICATIONS.
 */

/**
 * @file GoalSetter.C
 *    
 *
 * @author Dirk Steindorf
 * @date   2015/02/03
 */

#include <fw/MicroUnit.h>
#include <transform/Pose.h>
#include <fw/ChannelReadWrite.h>

using namespace mira;

namespace demopath { 

///////////////////////////////////////////////////////////////////////////////

/**
 * 
 */
class GoalSetter : public MicroUnit
{
MIRA_OBJECT(GoalSetter)

public:

	GoalSetter();

	template<typename Reflector>
	void reflect(Reflector& r)
	{
		MicroUnit::reflect(r);

		// TODO: reflect all parameters (members and properties) that specify the persistent state of the unit
		//r.property("Param1", mParam1, "First parameter of this unit with default value", 123.4f);
		//r.member("Param2", mParam2, setter(&UnitName::setParam2,this), "Second parameter with setter");
	}

protected:

	virtual void initialize();

private:

	// void onPoseChanged(ChannelRead<Pose2> pose);
	void onNewPose(ChannelRead<Pose2> pose);

private:

	//Channel<Img<>> mChannel;
	float translationTolerance; // meter
	float rotationTolerance; // radian
};

///////////////////////////////////////////////////////////////////////////////

GoalSetter::GoalSetter()
{
	// TODO: further initialization of members, etc.
	translationTolerance = 0.1f;
	rotationTolerance    = 0.1f;
}

void GoalSetter::initialize()
{
	// TODO: subscribe and publish all required channels
	//subscribe<Pose2>("Pose", &UnitName::onPoseChanged);
	//mChannel = publish<Img<>>("Image");
	subscribe<Pose2>("rosToMira", &GoalSetter::onNewPose);
	waitForService("/navigation/Pilot");
}

void BeaconDemo::onNewPose(ChannelRead<Pose2> pose)
{
    callService<void>("/navigation/Pilot", "setGoal", pose->value(), translationTolerance, rotationTolerance);
}

///////////////////////////////////////////////////////////////////////////////

}

MIRA_CLASS_SERIALIZATION(demopath::GoalSetter, mira::MicroUnit );
