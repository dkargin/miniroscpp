#include "minibag/bag_player.h"

namespace minibag
{

BagPlayer::BagPlayer(const std::string &fname) {
    bag.open(fname, minibag::bagmode::Read);
    miniros::Time::init();
    View v(bag);
    bag_start_ = v.getBeginTime();
    bag_end_ = v.getEndTime();
    last_message_time_ = miniros::Time(0);
    playback_speed_ = 1.0;
}

BagPlayer::~BagPlayer() {
    bag.close();
}

miniros::Time BagPlayer::get_time() {
    return last_message_time_;
}

void BagPlayer::set_start(const miniros::Time &start) {
    bag_start_ = start;
}

void BagPlayer::set_end(const miniros::Time &end) {
    bag_end_ = end;
}

void BagPlayer::set_playback_speed(double scale) {
  if (scale > 0.0)
    playback_speed_ = scale;
}

miniros::Time BagPlayer::real_time(const miniros::Time &msg_time) {
  return play_start_ + (msg_time - bag_start_) * (1 / playback_speed_);
}

void BagPlayer::start_play() {

    std::vector<std::string> topics;
    for (const auto& cb : cbs_)
        topics.push_back(cb.first);

    View view(bag, TopicQuery(topics), bag_start_, bag_end_);
    play_start_ = miniros::Time::now();

    for (MessageInstance const& m : view)
    {
        if (cbs_.find(m.getTopic()) == cbs_.end())
            continue;

        miniros::Time::sleepUntil(real_time(m.getTime()));

        last_message_time_ = m.getTime(); /* this is the recorded time */
        cbs_[m.getTopic()]->call(m);
    }
}

void BagPlayer::unregister_callback(const std::string &topic) {
    cbs_.erase(topic);
}

}

