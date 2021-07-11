#include "smart_home_analg/SoundFileManager.hpp"

#include <stdlib.h>
#include <assert.h>
#include <sndfile.h>
#include <SFML/Audio.hpp>
#include <ros/package.h>

namespace smart_home {

#define INST_CALC_FREQ_TMR() \
	nh.createTimer( \
		ros::Duration(current_fft_window), \
		&smart_home::SfManager::calc_frequencies_callback, \
		this, \
		false, \
		false \
	)

#define PLAYBACK_PREPARE() \
	ready = true; \
	calc_frequencies_tmr.start()

#define PLAYBACK_PLAY() sfpd->sound.play()

#define PLAYBACK_RESUME() \
	PLAYBACK_PLAY(); \
	calc_frequencies_tmr.start()

#define PLAYBACK_PAUSE() \
	sfpd->sound.pause(); \
	calc_frequencies_tmr.stop()

#define PLAYBACK_STOP() \
	sfpd->sound.stop(); \
	calc_frequencies_tmr.stop(); \
	sfpd->clear(); \
	ready = false

#define PLAYBACK_CONTINUE() \
	PLAYBACK_STOP(); \
	pop_next_playback()	

struct SfManager::SfPlaybackData
{
	std::string path;
	uint64_t num_frames;
	int num_channels;
	int sample_rate;

	int current_playback_chunk;
	std::vector<std::vector<double>> pending_playback_chunks;

	sf::SoundBuffer sound_buffer;
	sf::Sound sound;

	SfPlaybackData()
	{
		clear();
	}

	void clear()
	{
		path = "";
		num_frames = -1;
		num_channels = -1;
		sample_rate = -1;
		current_playback_chunk = -1;
		pending_playback_chunks.clear();
	}
};

void SfManager::queue_sound_file(const std::string& sf_path)
{
	queued_sound_file_paths.push(sf_path);
	if (!ready)
	{
		pop_next_playback();
	}
}

void SfManager::pop_next_playback()
{
	// Ignore if no next sound file is queued
	if (queued_sound_file_paths.empty())
	{
		return;
	}

	// Get full file path
	const std::string new_path = queued_sound_file_paths.front();
	queued_sound_file_paths.pop();
	sfpd->path = root_dir + "/" + new_path;

#define STR_ENDS_WITH(s,sx) (s.size() >= strlen(sx) && 0 == s.compare(s.size()-strlen(sx), strlen(sx), sx))

	// Create temporary file from MP3 to WAV, overwrite sfpd->path with temporary file
	if (STR_ENDS_WITH(sfpd->path, ".mp3"))
	{
		const size_t N = 256;
		const std::string command = std::string("bash \"") + ros::package::getPath("smart_home_analg") + "/bin/mp3_2_wav.sh\" \"" + sfpd->path + "\"";
		std::array<char,N> buffer;
		std::string result;

		FILE* pipe = popen(command.c_str(), "r");
		if (0 == pipe)
		{
			ROS_ERROR("Failed to start MP3 to WAV conversion script.");
			return;
		}

		// Capture output to get the tmp mp3 file
		while (0 != fgets(buffer.data(), N, pipe))
		{
			result += buffer.data();
		}
		result = result.substr(0, result.size()-1);

		// Command succeeded
		if (0 == pclose(pipe))
		{
			ROS_INFO("Created copy of '%s' to '%s'.", sfpd->path.c_str(), result.c_str());
			sfpd->path = result;
		}
		// Command failed
		else
		{
			ROS_ERROR("MP3 to WAV conversion script for %s failed (%s).", sfpd->path.c_str(), result.c_str());
			return;
		}
	}
	// Except for WAV files, nothing else is valid
	else if (!STR_ENDS_WITH(sfpd->path, ".wav"))
	{
		ROS_ERROR("Invalid file extension for '%s'.", sfpd->path.c_str());
		return;
	}

#undef STR_ENDS_WITH

	// Load file into file path
	if (!sfpd->sound_buffer.loadFromFile(sfpd->path))
	{
		ROS_ERROR("Failed to load sound buffer from file '%s'.", sfpd->path.c_str());
		return;
	}
	sfpd->sound.setBuffer(sfpd->sound_buffer);

	// Get the human-friendly name of the sound file
	std::string current_sf_name = new_path;
	const std::size_t directory = current_sf_name.rfind("/");
	const std::size_t extension = current_sf_name.rfind(".");
	if (std::string::npos != directory)
	{
		current_sf_name = current_sf_name.substr(directory+1);
	}
	if (std::string::npos != extension)
	{
		current_sf_name = current_sf_name.substr(0, extension);
	}

	// Read the file into buffer
	SF_INFO sfinfo;
	SNDFILE* file_in = sf_open(sfpd->path.c_str(), SFM_READ, &sfinfo);

	// Capture file parameters
	sfpd->num_frames = static_cast<uint64_t>(sfinfo.frames);
	sfpd->num_channels = static_cast<int>(sfinfo.channels);
	sfpd->sample_rate = static_cast<int>(sfinfo.samplerate);
	const uint64_t num_all_items = sfpd->num_frames * sfpd->num_channels;

	// Read sound file into buffer
	std::vector<double> item_buffer(num_all_items);
	const uint64_t items_read = sf_read_double(file_in, &item_buffer[0], num_all_items);
	sf_close(file_in);

	ROS_DEBUG(
		"frames=%lu, channels=%d, sample_rate=%d, num_all_items=%lu, items_read=%lu",
		sfpd->num_frames,
		sfpd->num_channels,
		sfpd->sample_rate,
		num_all_items,
		items_read
	);

	// Each chunk has Fs*w items in it (or is the entire sequence if no window size is given)
	const uint64_t frames_per_std_chunk_per_channel = (current_fft_window <= 0) ? sfpd->num_frames : sfpd->sample_rate * current_fft_window;
	const uint64_t frames_per_std_chunk = frames_per_std_chunk_per_channel * sfpd->num_channels;
	// Number of chunks is one less than the total items divided by items per chunk
	const uint64_t num_std_chunks = (sfpd->num_frames / frames_per_std_chunk_per_channel) - 1;
	// The remainder of the chunks
	const uint64_t frames_for_last_chunk = num_all_items - (frames_per_std_chunk * num_std_chunks);

	ROS_DEBUG(
		"frames_per_std_chunk=%lu, num_std_chunks=%lu, frames_for_last_chunk=%lu",
		frames_per_std_chunk,
		num_std_chunks,
		frames_for_last_chunk
	);

	// Queue every chunk of data
	sfpd->pending_playback_chunks.resize(num_std_chunks+1);
	for (int i = 0; i < sfpd->pending_playback_chunks.size(); i++)
	{
		const uint64_t items = (i == num_std_chunks) ? frames_for_last_chunk : frames_per_std_chunk;
		sfpd->pending_playback_chunks[i] = std::vector<double>(
			item_buffer.begin() + (i*frames_per_std_chunk),
			item_buffer.begin() + (i*frames_per_std_chunk) + items
		);
	}

	ROS_INFO("Finished queuing playback chunks for '%s'.", sfpd->path.c_str());
	PLAYBACK_PREPARE();
}

std::vector<float> SfManager::get_max_freqs(int chunk)
{
	assert(ready);
	return get_max_fft_freqs(
		do_1d_dft(sfpd->pending_playback_chunks[chunk]),
		sfpd->sample_rate,
		max_num_freqs
	);
}

SfManager::SfManager(
		const std::string root_dir,
		const int max_num_freqs,
		const float init_fft_window,
		const std::string sound_file_path_sub_topic,
		const std::string sound_file_path_list_sub_topic,
		const std::string fft_window_sub_topic,
		const std::string playback_command_sub_topic,
		const std::string playback_frequencies_pub_topic
) :
		root_dir(root_dir),
		max_num_freqs(max_num_freqs),
		current_fft_window(init_fft_window),
		ready(false),
		sfpd(new SfPlaybackData),
		playback_frequencies_msg(new smart_home_msgs::Float32Arr)
{
	assert(max_num_freqs > 0);
	sound_file_path_sub = nh.subscribe(
		sound_file_path_sub_topic,
		1,
		&smart_home::SfManager::sound_file_path_callback,
		this
	);
	sound_file_path_list_sub = nh.subscribe(
		sound_file_path_list_sub_topic,
		1,
		&smart_home::SfManager::sound_file_path_callback,
		this
	);
	fft_window_sub = nh.subscribe(
		fft_window_sub_topic,
		1,
		&smart_home::SfManager::fft_window_callback,
		this
	);
	playback_command_sub = nh.subscribe(
		playback_command_sub_topic,
		1,
		&smart_home::SfManager::playback_command_callback,
		this
	);
	playback_frequencies_pub = nh.advertise<smart_home_msgs::Float32Arr>(
		playback_frequencies_pub_topic,
		10
	);
	calc_frequencies_tmr = INST_CALC_FREQ_TMR();

	ROS_INFO("Initialized SoundFileAnalyzer.");
}
SfManager::~SfManager()
{
	delete sfpd;
}

void SfManager::sound_file_path_callback(const std_msgs::String& msg)
{
	queue_sound_file(msg.data);
}
void SfManager::sound_file_path_list_callback(const smart_home_msgs::StringArr& msg)
{
	for (std::vector<std::string>::const_iterator iter = msg.data.cbegin(); iter != msg.data.cend(); ++iter)
	{
		queue_sound_file(*iter);
	}
}
void SfManager::fft_window_callback(const std_msgs::Float32& msg)
{
	// Ignore if currently processing an audio file
	if (!ready)
	{
		current_fft_window = msg.data;
		calc_frequencies_tmr = INST_CALC_FREQ_TMR();
	}
}
void SfManager::playback_command_callback(const smart_home_msgs::PlaybackCommand& msg)
{
	switch (msg.cmd)
	{
		case smart_home_msgs::PlaybackCommand::STOP:
			PLAYBACK_STOP();
			break;
		case smart_home_msgs::PlaybackCommand::PAUSE:
			PLAYBACK_PAUSE();
			break;
		case smart_home_msgs::PlaybackCommand::RESUME:
			PLAYBACK_RESUME();
			break;
		case smart_home_msgs::PlaybackCommand::SKIP:
			PLAYBACK_CONTINUE();
			break;
	}
}
void SfManager::calc_frequencies_callback(const ros::TimerEvent& evt)
{
	if (!ready)
	{
		// Catch the case where a STOP command, etc. was requested the same spin cycle as a timer tick
		return;
	}

	const double start = ros::Time::now().toSec();
	playback_frequencies_msg->data = get_max_freqs(++sfpd->current_playback_chunk);
	playback_frequencies_pub.publish(playback_frequencies_msg);

	if (sfpd->current_playback_chunk == 0)
	{
		PLAYBACK_PLAY();
	}
	else if (sfpd->current_playback_chunk == sfpd->pending_playback_chunks.size()-1)
	{
		PLAYBACK_CONTINUE();
	}

	ROS_DEBUG(
		"Callback #%d/%lu is %f seconds behind, took %f seconds.",
		sfpd->current_playback_chunk,
		sfpd->pending_playback_chunks.size()-1,
		evt.current_real.toSec() - evt.current_expected.toSec(),
		ros::Time::now().toSec() - start
	);
}

#undef INST_CALC_FREQ_TMR

}
