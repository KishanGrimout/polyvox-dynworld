/*******************************************************************************
* The MIT License (MIT)
*
* Copyright (c) 2015 David Williams and Matthew Williams
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*******************************************************************************/

#ifndef __PolyVox_PagedVolume_H__
#define __PolyVox_PagedVolume_H__

#include "BaseVolume.h"
#include "Region.h"
#include "Vector.h"

#include <atomic>
#include <limits>
#include <cstdlib> //For abort()
#include <cstring> //For memcpy
#include <unordered_map>
#include <list>
#include <map>
#include <memory>
#include <stdexcept> //For invalid_argument
#include <vector>

namespace PolyVox
{
	class DummyRWLock
	{
	public:
		void readLock() const {}
		void readUnlock() const {}
		void writeLock() const {}
		void writeUnlock() const {}
		void upgradeLock() const {}
		void downgradeLock() const {}
		bool isLocked() const { return false; }

		template <typename T> static T postIncr(T& value) { return value++; }
		template <typename T> static T preIncr(T& value)  { return ++value; }
		template <typename T> static T postDecr(T& value) { return value--; }
		template <typename T> static T preDecr(T& value)  { return --value; }
	};

	template <typename AutoReadLockType>
	class AutoUpgradeLock
	{
		AutoReadLockType& m_readLock;
	public:
		AutoUpgradeLock(AutoReadLockType& readLock);
		~AutoUpgradeLock();
	};

	template <typename ResourceType>
	class AutoReadLock
	{
		const ResourceType* m_resource;
	public:
		AutoReadLock(const ResourceType* resourceToLock);
		~AutoReadLock();

		friend class AutoUpgradeLock<AutoReadLock<ResourceType>>;
	};

	template <typename ResourceType>
	class AutoWriteLock
	{
		const ResourceType* m_resource;
	public:
		AutoWriteLock(const ResourceType* resourceToLock);
		~AutoWriteLock();
	};

	enum class RegionStatus
	{
		Available,
		Locked,
		NotAvailable
	};

	/// The PagedVolume class provides a memory efficient method of storing voxel data while also allowing fast access and modification.
	/// This class provide a volume implementation which avoids storing all the data in memory at all times. Instead it breaks the volume
	/// down into a set of chunks and moves these into and out of memory on demand. This means it is much more memory efficient than the
	/// RawVolume, but may also be slower and is more complicated We encourage uses to work with RawVolume initially, and then switch to
	/// PagedVolume once they have a larger application and/or a better understanding of PolyVox.
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// 
	/// The PagedVolume makes use of a Pager which defines the source and/or destination for data paged into and out of memory. PolyVox
	/// comes with an example FilePager though users can also implement their own approaches. For example, the Pager could instead stream
	/// data from a network connection or generate it procedurally on demand.
	///
	/// A consequence of this paging approach is that (unlike the RawVolume) the PagedVolume does not need to have a predefined size. After
	/// the volume has been created you can begin acessing voxels anywhere in space and the required data will be created automatically.
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template <typename VoxelType, typename RWLockType = DummyRWLock>
	class PagedVolume : public BaseVolume<VoxelType>
	{
	public:
		/// The PagedVolume stores it data as a set of Chunk instances which can be loaded and unloaded as memory requirements dictate.
		class Chunk;
		/// The Pager class is responsible for the loading and unloading of Chunks, and can be subclassed by the user.
		class Pager;

		class Chunk
		{
			friend class PagedVolume;

		public:
			Chunk(Vector3DInt32 v3dPosition, uint16_t uSideLength, Pager* pPager = nullptr);
			~Chunk();

			VoxelType* getData(void) const;
			uint32_t getDataSizeInBytes(void) const;

			VoxelType getVoxel(uint32_t uXPos, uint32_t uYPos, uint32_t uZPos) const;
			VoxelType getVoxel(const Vector3DUint16& v3dPos) const;

			void setVoxel(uint32_t uXPos, uint32_t uYPos, uint32_t uZPos, VoxelType tValue);
			void setVoxel(const Vector3DUint16& v3dPos, VoxelType tValue);

			bool getDataModified() const;
			bool isLocked() const;

			// NOTE(dynworld): true while the pager is filling this chunk's
			// buffer with no volume/chunk locks held. Readers must wait for it
			// to clear (spin, without holding locks) before reading the buffer;
			// getVoxel/getChunk enforce this, so only code that knows it owns
			// the page-in window (the pager itself) may skip the check.
			bool isPagingInProgress() const
			{
				return m_bPagingInProgress.load(std::memory_order_acquire);
			}

			void changeLinearOrderingToMorton(void);
			void changeMortonOrderingToLinear(void);

			void readLock() const;
			void readUnlock() const;
			void writeLock() const;
			void writeUnlock() const;
			void upgradeLock() const;
			void downgradeLock() const;

		private:
			/// Private copy constructor to prevent accisdental copying
			Chunk(const Chunk& /*rhs*/) {};

			/// Private assignment operator to prevent accisdental copying
			Chunk& operator=(const Chunk& /*rhs*/) {};

			uint32_t calculateSizeInBytes(void);
			static uint32_t calculateSizeInBytes(uint32_t uSideLength);

			void pageIn();
			void pageOut();

			VoxelType* m_tData;

			Pager* m_pPager;

			// This is updated by the PagedVolume and used to discard the least recently used chunks.
			uint32_t m_uChunkLastAccessed;

			const uint16_t m_uSideLength;
			const uint8_t m_uSideLengthPower;

			// This is so we can tell whether a uncompressed chunk has to be recompressed and whether
			// a compressed chunk has to be paged back to disk, or whether they can just be discarded.
			bool m_bDataModified;

			// Note: Do we really need to store this position here as well as in the block maps?
			Vector3DInt32 m_v3dChunkSpacePosition;

			mutable RWLockType m_lock;

			// NOTE(dynworld): raised before the chunk is published into the
			// chunk array and cleared when the pager body completes, so readers
			// never observe uninitialized buffer contents. While it is set the
			// chunk write lock is NOT held - waiting readers spin on the flag
			// instead of blocking on the lock, so the wait does not hold the
			// volume internal lock read (the former contention chain).
			std::atomic<bool> m_bPagingInProgress{ false };
		};

		class IChunkCache
		{
		public:
			virtual ~IChunkCache() {}

			virtual std::weak_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> getChunkCached(const PagedVolume<VoxelType, RWLockType>* pVolume, int32_t iChunkX, int32_t iChunkY, int32_t iChunkZ) const = 0;
			virtual void setChunkCached(const PagedVolume<VoxelType, RWLockType>* pVolume, std::weak_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> pChunk, int32_t iChunkX, int32_t iChunkY, int32_t iChunkZ) = 0;
			virtual void dirtyCache() = 0;
		};

		class DefaultChunkCache : public IChunkCache
		{
		public:
			virtual std::weak_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> getChunkCached(const PagedVolume<VoxelType, RWLockType>* pVolume, int32_t iChunkX, int32_t iChunkY, int32_t iChunkZ) const override;
			virtual void setChunkCached(const PagedVolume<VoxelType, RWLockType>* pVolume, std::weak_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> pChunk, int32_t iChunkX, int32_t iChunkY, int32_t iChunkZ) override;
			virtual void dirtyCache() override;

		private:
			// Storing these properties individually has proved to be faster than keeping
			// them in a Vector3DInt32 as it avoids constructions and comparison overheads.
			// They are also at the start of the class in the hope that they will be pulled
			// into cache - I've got no idea if this actually makes a difference.
			int32_t m_v3dLastAccessedChunkX = 0;
			int32_t m_v3dLastAccessedChunkY = 0;
			int32_t m_v3dLastAccessedChunkZ = 0;
			std::weak_ptr<typename PagedVolume<VoxelType, RWLockType>::Chunk> m_pLastAccessedChunk;
		};

		/**
		* Users can override this class and provide an instance of the derived class to the PagedVolume constructor. This derived class
		* could then perform tasks such as compression and decompression of the data, and read/writing it to a file, database, network,
		* or other storage as appropriate. See FilePager for a simple example of such a derived class.
		*/
		class Pager
		{
		public:
			/// Constructor
			Pager() {};
			/// Destructor
			virtual ~Pager() {};

			virtual void pageIn(const Region& region, Chunk* pChunk) = 0;
			virtual void pageOut(const Region& region, Chunk* pChunk) = 0;
		};

		//There seems to be some descrepency between Visual Studio and GCC about how the following class should be declared.
		//There is a work around (see also See http://goo.gl/qu1wn) given below which appears to work on VS2010 and GCC, but
		//which seems to cause internal compiler errors on VS2008 when building with the /Gm 'Enable Minimal Rebuild' compiler
		//option. For now it seems best to 'fix' it with the preprocessor insstead, but maybe the workaround can be reinstated
		//in the future
		//typedef Volume<VoxelType> VolumeOfVoxelType; //Workaround for GCC/VS2010 differences.
		//class Sampler : public VolumeOfVoxelType::template Sampler< PagedVolume<VoxelType, RWLockType> >
#ifndef SWIG
#if defined(_MSC_VER)
		class Sampler : public BaseVolume<VoxelType>::Sampler< PagedVolume<VoxelType, RWLockType> > //This line works on VS2010
#else
		class Sampler : public BaseVolume<VoxelType>::template Sampler< PagedVolume<VoxelType, RWLockType> > //This line works on GCC
#endif
		{
		public:
			Sampler(PagedVolume<VoxelType, RWLockType>* volume);
			~Sampler();

			inline VoxelType getVoxel(void) const;

			void setPosition(const Vector3DInt32& v3dNewPos);
			void setPosition(int32_t xPos, int32_t yPos, int32_t zPos);
			inline bool setVoxel(VoxelType tValue);

			void movePositiveX(void);
			void movePositiveY(void);
			void movePositiveZ(void);

			void moveNegativeX(void);
			void moveNegativeY(void);
			void moveNegativeZ(void);

			inline VoxelType peekVoxel1nx1ny1nz(void) const;
			inline VoxelType peekVoxel1nx1ny0pz(void) const;
			inline VoxelType peekVoxel1nx1ny1pz(void) const;
			inline VoxelType peekVoxel1nx0py1nz(void) const;
			inline VoxelType peekVoxel1nx0py0pz(void) const;
			inline VoxelType peekVoxel1nx0py1pz(void) const;
			inline VoxelType peekVoxel1nx1py1nz(void) const;
			inline VoxelType peekVoxel1nx1py0pz(void) const;
			inline VoxelType peekVoxel1nx1py1pz(void) const;

			inline VoxelType peekVoxel0px1ny1nz(void) const;
			inline VoxelType peekVoxel0px1ny0pz(void) const;
			inline VoxelType peekVoxel0px1ny1pz(void) const;
			inline VoxelType peekVoxel0px0py1nz(void) const;
			inline VoxelType peekVoxel0px0py0pz(void) const;
			inline VoxelType peekVoxel0px0py1pz(void) const;
			inline VoxelType peekVoxel0px1py1nz(void) const;
			inline VoxelType peekVoxel0px1py0pz(void) const;
			inline VoxelType peekVoxel0px1py1pz(void) const;

			inline VoxelType peekVoxel1px1ny1nz(void) const;
			inline VoxelType peekVoxel1px1ny0pz(void) const;
			inline VoxelType peekVoxel1px1ny1pz(void) const;
			inline VoxelType peekVoxel1px0py1nz(void) const;
			inline VoxelType peekVoxel1px0py0pz(void) const;
			inline VoxelType peekVoxel1px0py1pz(void) const;
			inline VoxelType peekVoxel1px1py1nz(void) const;
			inline VoxelType peekVoxel1px1py0pz(void) const;
			inline VoxelType peekVoxel1px1py1pz(void) const;

		private:
			//Other current position information
			VoxelType* mCurrentVoxel;

			uint16_t m_uXPosInChunk;
			uint16_t m_uYPosInChunk;
			uint16_t m_uZPosInChunk;

			// This should ideally be const, but that prevent automatic generation of an assignment operator (https://goo.gl/Sn7KpZ).
			// We could provide one manually, but it's currently unused so there is no real test for if it works. I'm putting
			// together a new release at the moment so I'd rathern not make 'risky' changes.
			uint16_t m_uChunkSideLengthMinusOne;
		};

#endif // SWIG

		class AutoReadLockRegion
		{
			const PagedVolume<VoxelType, RWLockType>* m_volume;
			const Region& m_region;

		public:
			AutoReadLockRegion(const PagedVolume<VoxelType, RWLockType>* volume, const Region& region);
			~AutoReadLockRegion();
		};

		class AutoWriteLockRegion
		{
			const PagedVolume<VoxelType, RWLockType>* m_volume;
			const Region& m_region;

		public:
			AutoWriteLockRegion(const PagedVolume<VoxelType, RWLockType>* volume, const Region& region);
			~AutoWriteLockRegion();
		};

	public:
		/// Constructor for creating a fixed size volume.
		PagedVolume(Pager* pPager, uint32_t uTargetMemoryUsageInBytes = 256 * 1024 * 1024, uint16_t uChunkSideLength = 32, IChunkCache* pChunkCache = new DefaultChunkCache);
		/// Destructor
		~PagedVolume();

		/// Gets a voxel at the position given by <tt>x,y,z</tt> coordinates
		VoxelType getVoxel(int32_t uXPos, int32_t uYPos, int32_t uZPos) const;
		/// Gets a voxel at the position given by a 3D vector
		VoxelType getVoxel(const Vector3DInt32& v3dPos) const;

		/// Sets the voxel at the position given by <tt>x,y,z</tt> coordinates
		void setVoxel(int32_t uXPos, int32_t uYPos, int32_t uZPos, VoxelType tValue);
		/// Sets the voxel at the position given by a 3D vector
		void setVoxel(const Vector3DInt32& v3dPos, VoxelType tValue);

		// Queries the status of the region: Available if in memory, Locked if at least partially locked (load in progress for instance), NotAvailable if not all in memory
		RegionStatus getRegionStatus(const Region& regQuery);
		/// Tries to ensure that the voxels within the specified Region are loaded into memory.
		void prefetch(const Region& regPrefetch);
		void flush(const Region& regFlush);
		/// Removes all voxels from memory
		void flushAll();

		/// Calculates approximatly how many bytes of memory the volume is currently using.
		uint32_t calculateSizeInBytes(void);

		void readLock() const;
		void readUnlock() const;
		void writeLock() const;
		void writeUnlock() const;
		void upgradeLock() const;
		void downgradeLock() const;

		void readLockRegion(const Region& region) const;
		void readUnlockRegion(const Region& region) const;
		void writeLockRegion(const Region& region) const;
		void writeUnlockRegion(const Region& region) const;

	protected:
		/// These functions should never be called. Copying volumes by value would be expensive, and we want to prevent users from doing
		/// it by accident (such as when passing them as paramenters to functions). That said, there are times when you really do want to
		/// make a copy of a volume and in this case you should look at the VolumeResampler.
		/// \sa VolumeResampler

		/// Copy constructor
		PagedVolume(const PagedVolume& rhs) = delete;

		/// Assignment operator
		PagedVolume& operator=(const PagedVolume& rhs) = delete;

	private:
		template <typename Get, typename F>
		void applyOnChunks(const Region& region, F operation, Get getChunkF) const;
		uint32_t getPositionHash(int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const;
		std::shared_ptr<Chunk> tryGetChunk(int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const;
		std::shared_ptr<Chunk> getChunk(int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const;
		std::shared_ptr<Chunk> getExistingChunk(uint32_t iPosisionHash, int32_t uChunkX, int32_t uChunkY, int32_t uChunkZ) const;

		mutable IChunkCache* m_pChunkCache;

		mutable uint32_t m_uTimestamper = 0;

		uint32_t m_uChunkCountLimit = 0;

		// NOTE(dynworld): number of chunks currently in m_arrayChunks,
		// maintained incrementally under the internal write lock (creations,
		// evictions, flushes) so the O(uChunkArraySize) eviction scan only runs
		// when the limit is actually exceeded. Read under the internal read
		// lock by calculateSizeInBytes().
		mutable uint32_t m_uChunkCount = 0;

		// Chunks are stored in the following array which is used as a hash-table. Conventional wisdom is that such a hash-table
		// should not be more than half full to avoid conflicts, and a practical chunk size seems to be 64^3. With this configuration
		// there can be up to 32768*64^3 = 8 gigavoxels (with each voxel perhaps being many bytes). This should effectively make use 
		// of even high end machines. Of course, the user can choose to limit the memory usage in which case much less of the chunk 
		// array will actually be used. None-the-less, we have chosen to use a fixed size array (rather than a vector) as it appears to 
		// be slightly faster (probably due to the extra pointer indirection in a vector?) and the actual size of this array should
		// just be 1Mb or so.
		static const uint32_t uChunkArraySize = 65536;
		mutable std::shared_ptr< Chunk > m_arrayChunks[uChunkArraySize];

		// The size of the chunks
		const uint16_t m_uChunkSideLength;
		const uint8_t m_uChunkSideLengthPower;
		const int32_t m_iChunkMask;

		Pager* m_pPager = nullptr;

		mutable RWLockType m_volumeLock;
		mutable RWLockType m_internalLock;

		typedef PagedVolume<VoxelType, RWLockType> _Myt;
	};

}

#include "PagedVolume.inl"
#include "PagedVolumeChunk.inl"
#include "PagedVolumeSampler.inl"

#endif //__PolyVox_PagedVolume_H__
