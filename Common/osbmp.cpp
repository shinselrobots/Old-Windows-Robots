
#include "stdafx.h"
#include "osbmp.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


//-----------------------------------------------------------------------------
//
//	COffscreenDC::COffscreenDC
//
//	@mfunc This object implements an offscreen DC for the supplied screen DC.
//		You would then use this object to perform all of your GDI operations,
//		rendering into an offscreen bitmap.  When this object is deleted, it
//		then Blt's the offscreen bitmap into the supplied screen DC, preserving
//		any regions and palettes.  You would use this object to avoid flicker
//		erasing and rendering an image directly to the screen.
//
//-----------------------------------------------------------------------------
COffscreenDC::COffscreenDC( 
	CDC *pDC, // @parm The screen DC you want to buffer offscreen.
	CPalette *pPalette ) // @parmopt An optional palette you want to use for this DC.
{
	m_pDC = pDC;
	m_pPalette = pPalette;
	m_pOldPalette = NULL;
	m_bOwnDC = FALSE;

	Initialize();
}



void COffscreenDC::Initialize()
{
	// Create our offscreen DC
	CreateCompatibleDC( m_pDC ); // destructor release DC

	// Select the supplied palette if it exists
	if( NULL != m_pPalette ) 
	{
		m_pOldPalette = SelectPalette( m_pPalette, FALSE );
		RealizePalette();
	}

	// Get window size
	CWnd *pWnd = m_pDC->GetWindow();
	ASSERT( NULL != pWnd );
	pWnd->GetClientRect( m_rect );

	// Create offscreen bitmap
	m_osbmp.CreateCompatibleBitmap( m_pDC, m_rect.Width(), m_rect.Height() );
	m_oldbmp = SelectObject( &m_osbmp );
}



COffscreenDC::~COffscreenDC()
{
	// Get the currently selected region in offscreen DC, and select
	// that same region into the screen DC, if a region exists.
	CWnd *pWnd = m_pDC->GetWindow();
	if( NULL != pWnd )
	{
		CRgn rgn ;
		rgn.CreateRectRgn( 0, 0, 0, 0 );

		int dResult = ::GetWindowRgn( pWnd->GetSafeHwnd(), rgn );
		if( ( dResult != ERROR ) && ( dResult != NULLREGION ) )
			m_pDC->SelectClipRgn( &rgn, RGN_COPY );
	}

	// Blt the offscreen image in the screen DC.
	m_pDC->BitBlt( 0, 0, m_rect.Width(), m_rect.Height(), this, 0, 0, SRCCOPY ); 

	// GDI cleanup
	SelectObject( m_oldbmp );

	if( NULL != m_pOldPalette )
		SelectPalette( m_pOldPalette, TRUE );

	if( TRUE == m_bOwnDC )
		delete m_pDC;
}

