// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test availability of node interfaces constructors of message_filters::SubscriberBase.
 * \author Martin Pecka
 */

#include <message_filters/subscriber.hpp>

// DeprecatedTemplateParameter struct was added in the exact same commit that added the interfaces constructors.
struct Test : public message_filters::DeprecatedTemplateParameter
{
};
