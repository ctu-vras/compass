// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Test message_filters::SubscriberBase is a template or not.
 * \author Martin Pecka
 */

#include <message_filters/subscriber.hpp>

struct Test : public message_filters::SubscriberBase<>
{
};
